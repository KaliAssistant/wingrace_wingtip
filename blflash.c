/*
 * blflash.c - host tool for wingtip_boot.asm (ATtiny85 one-wire bootloader)
 *
 * Copyright (C) 2026 KaliAssistant <work.kaliassistant.github@gmail.com>
 * Copyright (C) 2026 HyperPROBE Labs. <hyperprobelabs@proton.me>
 *
 *   gcc -O2 -Wall -o blflash blflash.c
 *
 *   blflash [options] <command> [file]
 *
 *   commands:
 *     info              talk to the loader, print version / geometry
 *     flash  app.hex    flash an Intel-HEX (or raw .bin) application, verify, run
 *     verify app.hex    compare flash with an image
 *     read   out.bin    dump the whole 8 KB flash (128 pages)
 *     run               start the application
 *
 *   options:
 *     -p PORT   serial port. Optional: without it (or with "-p auto", or the
 *               BLFLASH_PORT environment variable) the tool probes the USB
 *               serial ports it finds (Linux /dev/ttyUSB* /dev/ttyACM*,
 *               macOS /dev/cu.usbserial* /dev/cu.usbmodem* ...) with a HELLO
 *               and uses the first one where the loader answers
 *     -l        list the candidate serial ports and exit
 *     -e        half-duplex adapter: every byte we send comes back on RX,
 *               discard that echo (single-wire / tied TX+RX adapters)
 *     -E        first send the app's "enter loader" sequence F0 B0 07
 *               (app must be running); without it power-cycle the chip
 *     -t SEC    how long to wait for the loader (default 15)
 *     -n        do not start the application after flashing
 *
 * Link: 9600 8N1 on the signal wire (the loader is fixed at 9600).
 * Application image: Intel HEX or raw binary at flash address 0 (as produced
 * by avra / avr-objcopy). Word 0 must be the application's "rjmp RESET"; this
 * tool moves it to word 0x0DFF and points word 0 at the loader (0xCDFF).
 * The application must therefore stay below 0x1BFE bytes.
 *
 * Frame:  host -> loader  B1 CMD ARG [64 data bytes for WRITE] CRCL CRCH
 *         loader -> host  B2 STATUS [payload if STATUS==0]     CRCL CRCH
 * CRC16 = avr-libc _crc_ccitt_update (CRC-16/MCRF4XX, init 0xFFFF) over CMD..DATA resp. STATUS..payload.
 */

#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <glob.h>
#include <errno.h>
#include <time.h>

#define MAGIC_H      0xB1
#define MAGIC_R      0xB2
#define CMD_HELLO    0x01
#define CMD_WRITE    0x02
#define CMD_READ     0x03
#define CMD_RUN      0x04
#define ST_OK        0
#define ST_RANGE     3

#define PAGE_BYTES   64
#define APP_PAGES    112                 /* pages 0..111 belong to the app   */
#define ALL_PAGES    128
#define FLASH_BYTES  (ALL_PAGES * PAGE_BYTES)
#define VEC_ADDR     0x1BFE              /* byte address of word 0x0DFF      */

static int fd = -1;
static int opt_echo = 0;
static int echo_to = 500;                /* ms to wait for our own echo */

/* ------------------------------------------------------------------------ */
static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "blflash: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    exit(1);
}

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void msleep(int ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* ------------------------------------------------------------------------ */
/* returns 0 ok / -1 failed (dies instead when fatal) */
static int ser_open(const char *dev, int fatal)
{
    struct termios t;
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (fatal) die("cannot open %s: %s", dev, strerror(errno));
        return -1;
    }
    if (tcgetattr(fd, &t) < 0) {
        if (fatal) die("%s: tcgetattr: %s", dev, strerror(errno));
        close(fd); fd = -1;
        return -1;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, B9600);
    cfsetospeed(&t, B9600);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        if (fatal) die("%s: tcsetattr: %s", dev, strerror(errno));
        close(fd); fd = -1;
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return 0;
}

static void ser_close(void)
{
    if (fd >= 0) close(fd);
    fd = -1;
}

/* read up to n bytes, give up after timeout_ms in total (select(): poll() is
 * unreliable on tty devices under macOS) */
static size_t ser_read(uint8_t *b, size_t n, int timeout_ms)
{
    size_t got = 0;
    long end = now_ms() + timeout_ms;
    while (got < n) {
        long left = end - now_ms();
        fd_set rs;
        struct timeval tv;
        int sr;
        if (left < 0) break;
        FD_ZERO(&rs);
        FD_SET(fd, &rs);
        tv.tv_sec = left / 1000;
        tv.tv_usec = (left % 1000) * 1000;
        sr = select(fd + 1, &rs, NULL, NULL, &tv);
        if (sr < 0 && errno == EINTR) continue;
        if (sr <= 0) break;
        ssize_t r = read(fd, b + got, n - got);
        if (r > 0) got += (size_t)r;
        else if (r < 0 && errno != EAGAIN && errno != EINTR)
            die("read: %s", strerror(errno));
    }
    return got;
}

static void ser_write(const uint8_t *b, size_t n)
{
    size_t done = 0;
    while (done < n) {
        fd_set ws;
        struct timeval tv = { 1, 0 };
        int sr;
        FD_ZERO(&ws);
        FD_SET(fd, &ws);
        sr = select(fd + 1, NULL, &ws, NULL, &tv);
        if (sr < 0 && errno == EINTR) continue;
        if (sr <= 0) die("write timeout");
        ssize_t r = write(fd, b + done, n - done);
        if (r > 0) done += (size_t)r;
        else if (r < 0 && errno != EAGAIN && errno != EINTR)
            die("write: %s", strerror(errno));
    }
    tcdrain(fd);
    if (opt_echo) {                       /* swallow our own echo */
        uint8_t tmp[256];
        size_t left = n;
        while (left) {
            size_t c = left > sizeof tmp ? sizeof tmp : left;
            size_t g = ser_read(tmp, c, echo_to);
            if (g == 0) break;
            left -= g;
        }
    }
}

/* ---- serial port discovery --------------------------------------------- */
#define MAXP    16
#define PATHLEN 256

static const char *port_globs[] = {
#ifdef __APPLE__
    "/dev/cu.usbserial*", "/dev/cu.usbmodem*",
    "/dev/cu.SLAB_USBtoUART*", "/dev/cu.wchusbserial*",
#else
    "/dev/ttyUSB*", "/dev/ttyACM*",
#endif
    NULL
};

static int list_ports(char list[][PATHLEN], int max)
{
    int n = 0, g, i;
    for (g = 0; port_globs[g]; g++) {
        glob_t gl;
        if (glob(port_globs[g], 0, NULL, &gl) != 0) continue;
        for (i = 0; i < (int)gl.gl_pathc && n < max; i++) {
            strncpy(list[n], gl.gl_pathv[i], PATHLEN - 1);
            list[n][PATHLEN - 1] = 0;
            n++;
        }
        globfree(&gl);
    }
    return n;
}

/* ------------------------------------------------------------------------ */
static uint16_t crc_upd(uint16_t crc, uint8_t d)      /* avr-libc _crc_ccitt_update */
{
    d ^= (uint8_t)(crc & 0xFF);
    d ^= (uint8_t)(d << 4);
    return (uint16_t)((((uint16_t)d << 8) | (crc >> 8)) ^ (uint8_t)(d >> 4)
                      ^ ((uint16_t)d << 3));
}

/* one request/response. returns STATUS (0..255), -1 no/short reply, -2 bad CRC */
static int xfer(uint8_t cmd, uint8_t arg, const uint8_t *data, size_t dlen,
                uint8_t *pay, size_t plen, int tmo_ms)
{
    uint8_t f[3 + PAGE_BYTES + 2], r[PAGE_BYTES + 2];
    size_t n = 0, i;
    uint16_t crc = 0xFFFF;
    uint8_t b;
    long end;

    f[n++] = MAGIC_H;
    f[n++] = cmd;
    f[n++] = arg;
    if (dlen) { memcpy(f + n, data, dlen); n += dlen; }
    for (i = 1; i < n; i++) crc = crc_upd(crc, f[i]);
    f[n++] = (uint8_t)crc;
    f[n++] = (uint8_t)(crc >> 8);

    tcflush(fd, TCIFLUSH);
    ser_write(f, n);

    end = now_ms() + tmo_ms;              /* hunt for the reply magic */
    for (;;) {
        long left = end - now_ms();
        if (left <= 0) return -1;
        if (ser_read(&b, 1, (int)left) != 1) return -1;
        if (b == MAGIC_R) break;
    }
    if (ser_read(&b, 1, 300) != 1) return -1;       /* status */
    {
        uint8_t st = b;
        size_t body = (st == ST_OK) ? plen : 0;
        if (ser_read(r, body + 2, 300) != body + 2) return -1;
        crc = crc_upd(0xFFFF, st);
        for (i = 0; i < body; i++) crc = crc_upd(crc, r[i]);
        if (r[body] != (uint8_t)crc || r[body + 1] != (uint8_t)(crc >> 8))
            return -2;
        if (body && pay) memcpy(pay, r, body);
        return st;
    }
}

/* ------------------------------------------------------------------------ */
/* ask for HELLO; 1 = loader answered */
static int probe_loader(void)
{
    uint8_t info[4];
    if (xfer(CMD_HELLO, 0, NULL, 0, info, 4, 120) != ST_OK) return 0;
    if (info[1] != PAGE_BYTES || info[2] != APP_PAGES)
        die("unexpected loader geometry (page %u bytes, %u app pages)",
            info[1], info[2]);
    printf("loader v%u, %u-byte pages, %u app pages\n", info[0], info[1], info[2]);
    return 1;
}

static void send_enter(void)
{
    static const uint8_t m[3] = { 0xF0, 0xB0, 0x07 };
    ser_write(m, sizeof m);
    msleep(50);
}

/* port == NULL: probe every candidate port until one answers (like esptool) */
static void connect_loader(const char *port, int enter, int wait_s)
{
    long t0 = now_ms(), end = t0 + wait_s * 1000L;
    int hinted = 0;

    if (!enter)
        fprintf(stderr, "waiting for the loader (power-cycle the board)...\n");
    if (port) ser_open(port, 1);
    while (now_ms() < end) {
        char list[MAXP][PATHLEN];
        int n = 0, i;
        if (port) {
            if (enter) send_enter();
            if (probe_loader()) return;
        } else {
            n = list_ports(list, MAXP);
            for (i = 0; i < n; i++) {
                if (ser_open(list[i], 0) < 0) continue;
                echo_to = 80;                 /* wrong ports must fail fast */
                if (enter) send_enter();
                if (probe_loader()) {
                    echo_to = 500;
                    printf("using %s\n", list[i]);
                    return;
                }
                ser_close();
            }
            echo_to = 500;
            if (!hinted && n == 0 && now_ms() > t0 + 1500) {
                fprintf(stderr, "no USB serial port found yet...\n");
                hinted = 1;
            }
        }
        if (enter && port && !hinted && now_ms() > t0 + 2000) {
            fprintf(stderr, "no answer yet - is the app running with BOOT_PRESENT=1?\n");
            hinted = 1;
        }
        msleep(50);
    }
    die("loader did not answer%s", port ? "" : " on any serial port (use -p PORT)");
}

static void write_page(int page, const uint8_t *d)
{
    int i, st = -1;
    for (i = 0; i < 4; i++) {
        st = xfer(CMD_WRITE, (uint8_t)page, d, PAGE_BYTES, NULL, 0, 1500);
        if (st == ST_OK) return;
        if (st == ST_RANGE) break;
        msleep(300);                      /* let a half-received frame time out */
    }
    die("write page %d failed (status %d)", page, st);
}

static void read_page(int page, uint8_t *d)
{
    int i, st = -1;
    for (i = 0; i < 4; i++) {
        st = xfer(CMD_READ, (uint8_t)page, NULL, 0, d, PAGE_BYTES, 1500);
        if (st == ST_OK) return;
        msleep(300);
    }
    die("read page %d failed (status %d)", page, st);
}

/* ------------------------------------------------------------------------ */
static uint8_t img[FLASH_BYTES];
static size_t img_hi;                     /* highest used byte + 1 */
static int last_page;                     /* last app page that holds data */

static int hexb(const char *s)
{
    int v = 0, i;
    for (i = 0; i < 2; i++) {
        char c = s[i];
        v <<= 4;
        if (c >= '0' && c <= '9') v |= c - '0';
        else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
        else return -1;
    }
    return v;
}

static void load_hex(FILE *f)
{
    char line[600];
    unsigned long base = 0;
    int eof = 0, ln = 0;
    while (fgets(line, sizeof line, f)) {
        int len, typ, sum, i;
        unsigned addr;
        ln++;
        if (line[0] != ':') continue;
        len = hexb(line + 1);
        addr = (unsigned)((hexb(line + 3) << 8) | hexb(line + 5));
        typ = hexb(line + 7);
        if (len < 0 || typ < 0 || addr > 0xFFFF) die("hex line %d malformed", ln);
        sum = len + (addr >> 8) + (addr & 0xFF) + typ;
        for (i = 0; i < len; i++) {
            int b = hexb(line + 9 + 2 * i);
            if (b < 0) die("hex line %d malformed", ln);
            sum += b;
            if (typ == 0) {
                unsigned long a = base + addr + (unsigned)i;
                if (a >= FLASH_BYTES) die("hex address 0x%lX beyond flash", a);
                if (a >= 0x1C00) die("hex line %d touches the loader area", ln);
                img[a] = (uint8_t)b;
                if (a + 1 > img_hi) img_hi = a + 1;
            }
        }
        sum += hexb(line + 9 + 2 * len);
        if (sum & 0xFF) die("hex line %d: bad checksum", ln);
        if (typ == 1) { eof = 1; break; }
        if (typ == 2) base = (unsigned long)((hexb(line + 9) << 8) | hexb(line + 11)) << 4;
        if (typ == 4) base = (unsigned long)((hexb(line + 9) << 8) | hexb(line + 11)) << 16;
    }
    if (!eof) die("hex file has no EOF record");
}

static void load_image(const char *path)
{
    size_t n = strlen(path);
    FILE *f = fopen(path, "rb");
    if (!f) die("cannot open %s: %s", path, strerror(errno));
    memset(img, 0xFF, sizeof img);
    img_hi = 0;
    if (n > 4 && !strcmp(path + n - 4, ".hex")) {
        load_hex(f);
    } else {
        img_hi = fread(img, 1, sizeof img, f);
        if (img_hi > 0x1C00) die("%s: binary too large", path);
    }
    fclose(f);
    if (img_hi == 0) die("%s: empty image", path);
    if (img_hi > VEC_ADDR)
        die("application too large: must end below byte 0x%X (word 0x0DFF is "
            "reserved for the moved reset vector)", VEC_ADDR);

    /* move the application's reset vector, point word 0 at the loader */
    {
        unsigned w0 = img[0] | (img[1] << 8);
        unsigned tgt, k;
        if ((w0 & 0xF000) != 0xC000) die("word 0 of the image is not an rjmp");
        if (w0 == 0xCDFF) die("image already points at the loader (patched?)");
        tgt = ((w0 & 0x0FFF) + 1) & 0x0FFF;        /* word address of RESET */
        k = (tgt - 0x0E00) & 0x0FFF;               /* rjmp from word 0x0DFF */
        img[VEC_ADDR]     = (uint8_t)((0xC000 | k) & 0xFF);
        img[VEC_ADDR + 1] = (uint8_t)((0xC000 | k) >> 8);
        img[0] = 0xFF;
        img[1] = 0xCD;
    }
    last_page = (int)((img_hi - 1) / PAGE_BYTES);
    if (last_page > 110) last_page = 110;
    printf("image: %zu bytes, pages 0..%d + 111\n", img_hi, last_page);
}

/* ------------------------------------------------------------------------ */
static int verify_image(void)
{
    uint8_t buf[PAGE_BYTES];
    int p, bad = 0;
    for (p = 0; p <= 111; p = (p == last_page ? 111 : p + 1)) {
        read_page(p, buf);
        if (memcmp(buf, img + p * PAGE_BYTES, PAGE_BYTES)) {
            fprintf(stderr, "mismatch in page %d\n", p);
            bad++;
        }
        if (p == 111) break;
    }
    return bad;
}

static void do_flash(int run)
{
    uint8_t blank[PAGE_BYTES];
    int p;

    memset(blank, 0xFF, sizeof blank);
    printf("invalidating application...\n");
    write_page(111, blank);               /* loader will not start a half app */
    for (p = 0; p <= last_page; p++) {
        printf("\rwriting page %3d/%d", p, last_page);
        fflush(stdout);
        write_page(p, img + p * PAGE_BYTES);
    }
    printf("\nwriting reset vector page...\n");
    write_page(111, img + 111 * PAGE_BYTES);
    printf("verifying...\n");
    if (verify_image()) die("verify FAILED");
    printf("verify OK\n");
    if (run) {
        int st = xfer(CMD_RUN, 0, NULL, 0, NULL, 0, 1500);
        if (st != ST_OK) die("run failed (status %d)", st);
        printf("application started\n");
    }
}

/* ------------------------------------------------------------------------ */
static void usage(void)
{
    fprintf(stderr,
        "usage: blflash [-p PORT|auto] [-l] [-e] [-E] [-t SEC] [-n] "
        "info | flash FILE | verify FILE | read OUT.bin | run\n");
    exit(2);
}

int main(int argc, char **argv)
{
    const char *port = getenv("BLFLASH_PORT"), *cmd, *file = NULL;
    int enter = 0, wait_s = 15, run = 1, i;

    for (i = 1; i < argc && argv[i][0] == '-'; i++) {
        if (!strcmp(argv[i], "-p") && i + 1 < argc) port = argv[++i];
        else if (!strcmp(argv[i], "-l")) {
            char list[MAXP][PATHLEN];
            int n = list_ports(list, MAXP), k;
            for (k = 0; k < n; k++) puts(list[k]);
            if (!n) fprintf(stderr, "no USB serial ports found\n");
            return n ? 0 : 1;
        }
        else if (!strcmp(argv[i], "-e")) opt_echo = 1;
        else if (!strcmp(argv[i], "-E")) enter = 1;
        else if (!strcmp(argv[i], "-n")) run = 0;
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) wait_s = atoi(argv[++i]);
        else usage();
    }
    if (port && (!*port || !strcmp(port, "auto"))) port = NULL;
    if (i >= argc) usage();
    cmd = argv[i++];
    if (i < argc) file = argv[i];

    if (!strcmp(cmd, "flash") || !strcmp(cmd, "verify")) {
        if (!file) usage();
        load_image(file);                 /* fail early, before touching the port */
    } else if (!strcmp(cmd, "read")) {
        if (!file) usage();
    } else if (strcmp(cmd, "info") && strcmp(cmd, "run")) {
        usage();
    }

    connect_loader(port, enter, wait_s);

    if (!strcmp(cmd, "flash")) {
        do_flash(run);
    } else if (!strcmp(cmd, "verify")) {
        if (verify_image()) die("verify FAILED");
        printf("verify OK\n");
    } else if (!strcmp(cmd, "read")) {
        static uint8_t dump[FLASH_BYTES];
        FILE *f;
        for (i = 0; i < ALL_PAGES; i++) {
            printf("\rreading page %3d/%d", i, ALL_PAGES - 1);
            fflush(stdout);
            read_page(i, dump + i * PAGE_BYTES);
        }
        printf("\n");
        f = fopen(file, "wb");
        if (!f || fwrite(dump, 1, sizeof dump, f) != sizeof dump)
            die("cannot write %s", file);
        fclose(f);
    } else if (!strcmp(cmd, "run")) {
        int st = xfer(CMD_RUN, 0, NULL, 0, NULL, 0, 1500);
        if (st != ST_OK) die("run failed (status %d)", st);
        printf("application started\n");
    }
    return 0;
}
