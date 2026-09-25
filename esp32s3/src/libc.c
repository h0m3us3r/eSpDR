/*
 * The few C library routines this image needs. The ROM provides these too,
 * but ROM code must not run while a capture overwrites the ROM's working
 * memory in capture bank 3, so the linker uses these instead.
 */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

void *memset(void *destination, int value, size_t size)
{
    unsigned char *d = destination;
    for (size_t i = 0; i < size; i++)
        d[i] = (unsigned char)value;
    return destination;
}

void *memcpy(void *destination, const void *source, size_t size)
{
    unsigned char *d = destination;
    const unsigned char *s = source;
    for (size_t i = 0; i < size; i++)
        d[i] = s[i];
    return destination;
}

void *memmove(void *destination, const void *source, size_t size)
{
    unsigned char *d = destination;
    const unsigned char *s = source;
    if (d < s) {
        for (size_t i = 0; i < size; i++)
            d[i] = s[i];
    } else {
        while (size) {
            size--;
            d[size] = s[size];
        }
    }
    return destination;
}

/*
 * Minimal sprintf for the PHY library's diagnostic strings: %s %c %d %i %u
 * %x %X with optional zero padding, width and l/ll length modifiers.
 */
int sprintf(char *out, const char *format, ...)
{
    char *start = out;
    va_list args;
    va_start(args, format);
    while (*format) {
        if (*format != '%') {
            *out++ = *format++;
            continue;
        }
        format++;
        char pad = ' ';
        if (*format == '0') {
            pad = '0';
            format++;
        }
        unsigned width = 0;
        while (*format >= '0' && *format <= '9')
            width = width * 10 + (unsigned)(*format++ - '0');
        unsigned longs = 0;
        while (*format == 'l') {
            longs++;
            format++;
        }
        char type = *format++;
        if (type == '%') {
            *out++ = '%';
            continue;
        }
        if (type == 's') {
            for (const char *s = va_arg(args, const char *); *s; s++)
                *out++ = *s;
            continue;
        }
        if (type == 'c') {
            *out++ = (char)va_arg(args, int);
            continue;
        }
        if (type != 'd' && type != 'i' && type != 'u' && type != 'x' && type != 'X') {
            *out++ = '?';
            continue;
        }
        uint64_t value = longs > 1 ? va_arg(args, unsigned long long) : va_arg(args, unsigned);
        if (type == 'd' || type == 'i') {
            int64_t signed_value = longs > 1 ? (int64_t)value : (int32_t)value;
            if (signed_value < 0) {
                *out++ = '-';
                value = 0 - (uint64_t)signed_value;
            }
        }
        unsigned radix = (type == 'x' || type == 'X') ? 16 : 10;
        const char *digits = type == 'X' ? "0123456789ABCDEF" : "0123456789abcdef";
        char buffer[32];
        unsigned n = 0;
        do {
            buffer[n++] = digits[value % radix];
            value /= radix;
        } while (value);
        while (width > n) {
            *out++ = pad;
            width--;
        }
        while (n)
            *out++ = buffer[--n];
    }
    *out = 0;
    va_end(args);
    return (int)(out - start);
}
