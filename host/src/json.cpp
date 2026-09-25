#include "json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace {

struct Reader {
    const std::string &text;
    size_t at = 0;

    [[noreturn]] void fail(const char *what) const
    {
        throw std::runtime_error(std::string("bad message: ") + what + " at " + std::to_string(at));
    }
    void space()
    {
        while (at < text.size() && (text[at] == ' ' || text[at] == '\t' || text[at] == '\n' || text[at] == '\r'))
            ++at;
    }
    bool take(char c)
    {
        space();
        if (at < text.size() && text[at] == c) {
            ++at;
            return true;
        }
        return false;
    }
    void expect(char c)
    {
        if (!take(c)) fail("unexpected character");
    }
    bool word(const char *w)
    {
        size_t n = std::char_traits<char>::length(w);
        if (text.compare(at, n, w) != 0) return false;
        at += n;
        return true;
    }
    void utf8(std::string &out, unsigned code)
    {
        if (code < 0x80) {
            out += char(code);
        } else if (code < 0x800) {
            out += char(0xC0 | code >> 6);
            out += char(0x80 | (code & 63));
        } else {
            out += char(0xE0 | code >> 12);
            out += char(0x80 | (code >> 6 & 63));
            out += char(0x80 | (code & 63));
        }
    }
    std::string string()
    {
        expect('"');
        std::string out;
        while (at < text.size() && text[at] != '"') {
            char c = text[at++];
            if (c != '\\') {
                out += c;
                continue;
            }
            if (at >= text.size()) fail("unterminated escape");
            char e = text[at++];
            switch (e) {
            case '"': case '\\': case '/': out += e; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                if (at + 4 > text.size()) fail("short \\u escape");
                utf8(out, unsigned(std::strtoul(text.substr(at, 4).c_str(), nullptr, 16)));
                at += 4;
                break;
            }
            default: fail("unknown escape");
            }
        }
        if (!take('"')) fail("unterminated string");
        return out;
    }
    JsonValue value()
    {
        space();
        JsonValue v;
        if (at >= text.size()) fail("missing value");
        char c = text[at];
        if (c == '"') {
            v.kind = JsonValue::String;
            v.string = string();
        } else if (word("true")) {
            v.kind = JsonValue::Boolean;
            v.boolean = true;
        } else if (word("false")) {
            v.kind = JsonValue::Boolean;
        } else if (word("null")) {
            v.kind = JsonValue::Null;
        } else if (c == '-' || (c >= '0' && c <= '9')) {
            char *end = nullptr;
            v.kind = JsonValue::Number;
            v.number = std::strtod(text.c_str() + at, &end);
            at = size_t(end - text.c_str());
            if (!std::isfinite(v.number)) fail("number out of range");
        } else {
            fail("unsupported value");
        }
        return v;
    }
};

}  // namespace

JsonObject parse_json_object(const std::string &text)
{
    Reader r{text};
    JsonObject object;
    r.expect('{');
    if (!r.take('}')) {
        do {
            r.space();
            std::string key = r.string();
            r.expect(':');
            object[key] = r.value();
        } while (r.take(','));
        r.expect('}');
    }
    r.space();
    if (r.at != text.size()) r.fail("trailing characters");
    return object;
}

std::string json_quote(const std::string &text)
{
    std::string out = "\"";
    for (unsigned char c : text) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += char(c);
        } else if (c == '\n') {
            out += "\\n";
        } else if (c < 0x20) {
            char escape[8];
            std::snprintf(escape, sizeof(escape), "\\u%04x", c);
            out += escape;
        } else {
            out += char(c);
        }
    }
    return out + "\"";
}

std::string json_number(double value)
{
    if (!std::isfinite(value)) return "null";
    char text[40];
    if (value == std::floor(value) && std::fabs(value) < 9e15) std::snprintf(text, sizeof(text), "%.0f", value);
    else std::snprintf(text, sizeof(text), "%.15g", value);
    return text;
}
