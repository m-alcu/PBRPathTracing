// tiny_yaml.hpp — minimal YAML parser, stb-style single-header library
//
// USAGE
//   In exactly ONE .cpp file before including this header:
//       #define TINY_YAML_IMPLEMENTATION
//       #include "tiny_yaml/tiny_yaml.hpp"
//   All other files just include normally (no #define).
//
// SUPPORTED YAML SUBSET
//   - Block mappings  (key: value, indentation-based)
//   - Block sequences (- item)
//   - Flow sequences  ([a, b, c])
//   - Scalars: string (quoted or bare), float, double, int, bool
//   - Inline comments (# ...)
//   - Quoted strings  ("..." or '...')
//
// API (compatible with yaml-cpp drop-in for this subset)
//   YAML::Node root = YAML::LoadFile("scene.yaml");
//   YAML::Node root = YAML::Load(std::string);
//   root["key"]          // map lookup — returns Null node if missing
//   root[i]              // sequence index
//   root.as<float>()     // scalar conversion
//   if (root["key"])     // existence test
//   for (auto& n : root) // iterate sequence items

#pragma once
#include <string>
#include <vector>
#include <stdexcept>

namespace YAML {

// ---------------------------------------------------------------------------
// Exception
// ---------------------------------------------------------------------------
class Exception : public std::runtime_error {
public:
    explicit Exception(const std::string& msg) : std::runtime_error(msg) {}
};

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------
enum class NodeType { Null, Scalar, Sequence, Map };

class Node {
public:
    // Map storage uses two parallel vectors to avoid the incomplete-type
    // problem that a nested struct { Node value; } would cause.
    NodeType             type     = NodeType::Null;
    std::string          scalar_;          // NodeType::Scalar
    std::vector<Node>    seq_;             // NodeType::Sequence
    std::vector<Node>    mapVals_;         // NodeType::Map — values
    std::vector<std::string> mapKeys_;     // NodeType::Map — keys (parallel)

    Node() = default;

    // True when node is not Null
    explicit operator bool() const { return type != NodeType::Null; }

    // Map access — returns Null node if key is absent
    Node operator[](const std::string& key) const {
        if (type != NodeType::Map) return {};
        for (std::size_t i = 0; i < mapKeys_.size(); ++i)
            if (mapKeys_[i] == key) return mapVals_[i];
        return {};
    }
    Node operator[](const char* key) const { return (*this)[std::string(key)]; }

    // Sequence access
    Node operator[](std::size_t i) const {
        if (type != NodeType::Sequence || i >= seq_.size()) return {};
        return seq_[i];
    }
    Node operator[](int i) const {
        return (*this)[static_cast<std::size_t>(i)];
    }

    // Scalar conversion (specialised below)
    template<typename T> T as() const;

    // Sequence iteration
    std::vector<Node>::const_iterator begin() const { return seq_.begin(); }
    std::vector<Node>::const_iterator end()   const { return seq_.end(); }
    std::size_t size() const {
        if (type == NodeType::Sequence) return seq_.size();
        if (type == NodeType::Map)      return mapKeys_.size();
        return 0;
    }

    // Internal helper used by the parser
    void mapPush(std::string key, Node val) {
        mapKeys_.push_back(std::move(key));
        mapVals_.push_back(std::move(val));
    }
};

// --- Scalar specialisations ------------------------------------------------

template<> inline std::string Node::as<std::string>() const {
    if (type != NodeType::Scalar)
        throw Exception("tiny_yaml: as<string> on non-scalar node");
    return scalar_;
}
template<> inline float Node::as<float>() const {
    if (type != NodeType::Scalar)
        throw Exception("tiny_yaml: as<float> on non-scalar node");
    try { return std::stof(scalar_); }
    catch (...) { throw Exception("tiny_yaml: cannot convert '" + scalar_ + "' to float"); }
}
template<> inline double Node::as<double>() const {
    if (type != NodeType::Scalar)
        throw Exception("tiny_yaml: as<double> on non-scalar node");
    try { return std::stod(scalar_); }
    catch (...) { throw Exception("tiny_yaml: cannot convert '" + scalar_ + "' to double"); }
}
template<> inline int Node::as<int>() const {
    if (type != NodeType::Scalar)
        throw Exception("tiny_yaml: as<int> on non-scalar node");
    try { return std::stoi(scalar_); }
    catch (...) { throw Exception("tiny_yaml: cannot convert '" + scalar_ + "' to int"); }
}
template<> inline bool Node::as<bool>() const {
    if (type != NodeType::Scalar)
        throw Exception("tiny_yaml: as<bool> on non-scalar node");
    if (scalar_ == "true"  || scalar_ == "True"  || scalar_ == "TRUE"  ||
        scalar_ == "yes"   || scalar_ == "Yes"   || scalar_ == "YES"   ||
        scalar_ == "on"    || scalar_ == "On"    || scalar_ == "ON")
        return true;
    if (scalar_ == "false" || scalar_ == "False" || scalar_ == "FALSE" ||
        scalar_ == "no"    || scalar_ == "No"    || scalar_ == "NO"    ||
        scalar_ == "off"   || scalar_ == "Off"   || scalar_ == "OFF")
        return false;
    throw Exception("tiny_yaml: cannot convert '" + scalar_ + "' to bool");
}

// ---------------------------------------------------------------------------
// Entry points (defined in TINY_YAML_IMPLEMENTATION)
// ---------------------------------------------------------------------------
Node Load(const std::string& input);
Node LoadFile(const std::string& path);

} // namespace YAML


// =============================================================================
// I M P L E M E N T A T I O N
// =============================================================================
#ifdef TINY_YAML_IMPLEMENTATION

#include <fstream>
#include <sstream>

namespace YAML {
namespace detail {

// ---------------------------------------------------------------------------
// String helpers
// ---------------------------------------------------------------------------

static std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    auto a = s.find_first_not_of(ws);
    if (a == std::string::npos) return {};
    auto b = s.find_last_not_of(ws);
    return s.substr(a, b - a + 1);
}

// Strip surrounding quotes (" or ')
static std::string unquote(std::string s) {
    if (s.size() >= 2 &&
        ((s.front() == '"'  && s.back() == '"') ||
         (s.front() == '\'' && s.back() == '\'')))
        return s.substr(1, s.size() - 2);
    return s;
}

// Find the ':' that acts as key-value separator (not inside quotes/brackets)
static std::size_t findColon(const std::string& s) {
    bool inQ = false; char qc = 0;
    int  depth = 0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (inQ) {
            if (c == qc) inQ = false;
        } else if (c == '"' || c == '\'') {
            inQ = true; qc = c;
        } else if (c == '[' || c == '{') {
            ++depth;
        } else if (c == ']' || c == '}') {
            --depth;
        } else if (c == ':' && depth == 0) {
            // Must be followed by space, tab, or end-of-string
            if (i + 1 == s.size() || s[i + 1] == ' ' || s[i + 1] == '\t')
                return i;
        }
    }
    return std::string::npos;
}

// ---------------------------------------------------------------------------
// Flow-sequence parser  "[a, b, c]"
// ---------------------------------------------------------------------------
static Node parseFlowSeq(const std::string& s) {
    Node n; n.type = NodeType::Sequence;
    auto beg = s.find('[');
    auto end = s.rfind(']');
    if (beg == std::string::npos || end == std::string::npos || end <= beg)
        return n;

    std::string inner = s.substr(beg + 1, end - beg - 1);
    std::string cur;
    bool inQ = false; char qc = 0;
    int  depth = 0;

    auto flush = [&]() {
        auto t = trim(cur);
        if (!t.empty()) {
            Node item; item.type = NodeType::Scalar;
            item.scalar_ = unquote(t);
            n.seq_.push_back(std::move(item));
        }
        cur.clear();
    };

    for (char c : inner) {
        if (inQ) {
            cur += c;
            if (c == qc) inQ = false;
        } else if (c == '"' || c == '\'') {
            inQ = true; qc = c; cur += c;
        } else if (c == '[' || c == '{') {
            ++depth; cur += c;
        } else if (c == ']' || c == '}') {
            --depth; cur += c;
        } else if (c == ',' && depth == 0) {
            flush();
        } else {
            cur += c;
        }
    }
    flush();
    return n;
}

static Node makeScalar(const std::string& s) {
    Node n; n.type = NodeType::Scalar;
    n.scalar_ = unquote(trim(s));
    return n;
}

// ---------------------------------------------------------------------------
// Tokeniser
// ---------------------------------------------------------------------------
struct RawLine {
    int         lineNum;
    int         indent;
    std::string content; // leading spaces stripped, trailing comments stripped
};

static std::vector<RawLine> tokenize(const std::string& src) {
    std::vector<RawLine> out;
    std::istringstream ss(src);
    std::string line;
    int ln = 0;

    while (std::getline(ss, line)) {
        ++ln;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // Measure indent (spaces only — tabs are invalid YAML indentation)
        int indent = 0;
        while (indent < (int)line.size() && line[indent] == ' ') ++indent;
        std::string content = line.substr(indent);

        // Strip inline comments (respect quotes)
        {
            bool inQ = false; char qc = 0;
            for (std::size_t i = 0; i < content.size(); ++i) {
                char c = content[i];
                if (inQ) { if (c == qc) inQ = false; }
                else if (c == '"' || c == '\'') { inQ = true; qc = c; }
                else if (c == '#') {
                    if (i == 0 || content[i - 1] == ' ' || content[i - 1] == '\t') {
                        content = content.substr(0, i);
                        break;
                    }
                }
            }
            content = trim(content);
        }

        if (content.empty()) continue;
        out.push_back({ln, indent, content});
    }
    return out;
}

// ---------------------------------------------------------------------------
// Recursive tree builder — forward declarations
// ---------------------------------------------------------------------------
static Node buildNode   (const std::vector<RawLine>&, int& pos, int parentIndent);
static Node buildMapNode(const std::vector<RawLine>&, int& pos, int myIndent);
static Node buildSeqNode(const std::vector<RawLine>&, int& pos, int myIndent);

// ---------------------------------------------------------------------------
// Build a map node: reads all consecutive lines at exactly myIndent that look
// like "key: value" pairs.
// ---------------------------------------------------------------------------
static Node buildMapNode(const std::vector<RawLine>& lines, int& pos, int myIndent) {
    Node n; n.type = NodeType::Map;

    while (pos < (int)lines.size() && lines[pos].indent == myIndent) {
        const RawLine& l = lines[pos];

        auto ci = findColon(l.content);
        if (ci == std::string::npos) break; // not a key:value — stop

        std::string key = unquote(trim(l.content.substr(0, ci)));
        std::string val = (ci + 1 < l.content.size())
                          ? trim(l.content.substr(ci + 1)) : "";

        Node child;
        if (val.empty()) {
            ++pos;
            if (pos < (int)lines.size() && lines[pos].indent > myIndent)
                child = buildNode(lines, pos, myIndent);
        } else if (val.front() == '[') {
            child = parseFlowSeq(val);
            ++pos;
        } else {
            child = makeScalar(val);
            ++pos;
        }
        n.mapPush(std::move(key), std::move(child));
    }
    return n;
}

// ---------------------------------------------------------------------------
// Build a sequence node: reads all consecutive "- ..." lines at myIndent.
// ---------------------------------------------------------------------------
static Node buildSeqNode(const std::vector<RawLine>& lines, int& pos, int myIndent) {
    Node n; n.type = NodeType::Sequence;

    while (pos < (int)lines.size() && lines[pos].indent == myIndent) {
        const RawLine& l = lines[pos];

        bool isSeq = l.content[0] == '-' &&
                     (l.content.size() == 1 ||
                      l.content[1] == ' '   ||
                      l.content[1] == '\t');
        if (!isSeq) break;

        std::string rest = (l.content.size() > 2) ? trim(l.content.substr(2)) : "";
        int itemIndent = myIndent + 2;

        Node item;
        if (rest.empty()) {
            ++pos;
            if (pos < (int)lines.size() && lines[pos].indent > myIndent)
                item = buildNode(lines, pos, myIndent);

        } else if (rest.front() == '[') {
            item = parseFlowSeq(rest);
            ++pos;

        } else {
            auto ci = findColon(rest);
            if (ci != std::string::npos) {
                // Inline map entry — first key is on the same line as "-"
                item.type = NodeType::Map;
                std::string key = unquote(trim(rest.substr(0, ci)));
                std::string val = (ci + 1 < rest.size())
                                  ? trim(rest.substr(ci + 1)) : "";

                Node firstVal;
                if (val.empty()) {
                    ++pos;
                    if (pos < (int)lines.size() && lines[pos].indent > myIndent)
                        firstVal = buildNode(lines, pos, myIndent);
                } else if (val.front() == '[') {
                    firstVal = parseFlowSeq(val);
                    ++pos;
                } else {
                    firstVal = makeScalar(val);
                    ++pos;
                }
                item.mapPush(std::move(key), std::move(firstVal));

                // Read remaining sibling keys indented at itemIndent
                while (pos < (int)lines.size() && lines[pos].indent == itemIndent) {
                    const RawLine& cl = lines[pos];
                    auto cci = findColon(cl.content);
                    if (cci == std::string::npos) break;

                    std::string ckey = unquote(trim(cl.content.substr(0, cci)));
                    std::string cval = (cci + 1 < cl.content.size())
                                       ? trim(cl.content.substr(cci + 1)) : "";

                    Node cchild;
                    if (cval.empty()) {
                        ++pos;
                        if (pos < (int)lines.size() && lines[pos].indent > itemIndent)
                            cchild = buildNode(lines, pos, itemIndent);
                    } else if (cval.front() == '[') {
                        cchild = parseFlowSeq(cval);
                        ++pos;
                    } else {
                        cchild = makeScalar(cval);
                        ++pos;
                    }
                    item.mapPush(std::move(ckey), std::move(cchild));
                }

            } else {
                item = makeScalar(rest);
                ++pos;
            }
        }
        n.seq_.push_back(std::move(item));
    }
    return n;
}

// ---------------------------------------------------------------------------
// Dispatch: decide whether to build a map, sequence, or scalar.
// parentIndent is the indent of the surrounding block; the new block must be
// strictly deeper.
// ---------------------------------------------------------------------------
static Node buildNode(const std::vector<RawLine>& lines, int& pos, int parentIndent) {
    if (pos >= (int)lines.size()) return {};
    int myIndent = lines[pos].indent;
    if (myIndent <= parentIndent) return {};

    const RawLine& l = lines[pos];

    if (l.content[0] == '-' &&
        (l.content.size() == 1 || l.content[1] == ' ' || l.content[1] == '\t'))
        return buildSeqNode(lines, pos, myIndent);

    if (findColon(l.content) != std::string::npos)
        return buildMapNode(lines, pos, myIndent);

    return makeScalar(l.content);
}

} // namespace detail

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

Node Load(const std::string& input) {
    auto lines = detail::tokenize(input);
    if (lines.empty()) return {};

    int pos       = 0;
    int topIndent = lines[0].indent;
    const auto& l0 = lines[0];

    bool isSeq = l0.content[0] == '-' &&
                 (l0.content.size() == 1 || l0.content[1] == ' ' || l0.content[1] == '\t');

    return isSeq ? detail::buildSeqNode(lines, pos, topIndent)
                 : detail::buildMapNode(lines, pos, topIndent);
}

Node LoadFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw Exception("tiny_yaml: cannot open '" + path + "'");
    std::string content{std::istreambuf_iterator<char>(f),
                        std::istreambuf_iterator<char>()};
    return Load(content);
}

} // namespace YAML

#endif // TINY_YAML_IMPLEMENTATION
