#include "Parser.hpp"

namespace wmtk::components::simwild::expression_parser {

ExpressionPtr Parser::parse()
{
    auto expr = parseOr();

    skipWhitespace();

    if (m_pos != m_input.size()) {
        log_and_throw_error("Unexpected trailing input");
    }

    return expr;
}

ExpressionPtr Parser::parseOr()
{
    auto left = parseAnd();

    while (true) {
        skipWhitespace();

        if (match('|')) {
            auto right = parseAnd();
            left = std::make_shared<OrExpr>(left, right);
        } else {
            break;
        }
    }

    return left;
}

ExpressionPtr Parser::parseAnd()
{
    auto left = parseUnary();

    while (true) {
        skipWhitespace();

        if (match('&')) {
            auto right = parseUnary();
            left = std::make_shared<AndExpr>(left, right);
        } else {
            break;
        }
    }

    return left;
}

ExpressionPtr Parser::parseUnary()
{
    skipWhitespace();

    if (match('!')) {
        return std::make_shared<NotExpr>(parseUnary());
    }

    return parsePrimary();
}

ExpressionPtr Parser::parsePrimary()
{
    skipWhitespace();

    if (match('(')) {
        auto expr = parseOr();

        skipWhitespace();

        if (!match(')')) {
            log_and_throw_error("Expected ')' in expression {}, at position {}", m_input, m_pos);
        }

        return expr;
    }

    std::string ident = parseIdentifier();

    if (ident.empty()) {
        log_and_throw_error("Expected identifier in expression {}, at position {}", m_input, m_pos);
    }

    int64_t tag_id = -1;
    if (ident == "_") {
        // treat "_" as empty tag
        return std::make_shared<EmptyExpr>();
    }

    auto it = m_tag_name_to_id.find(ident);
    if (it != m_tag_name_to_id.end()) {
        tag_id = it->second;
    } else {
        logger().warn("Tag name {} does not exist!", ident);
    }

    return std::make_shared<TagExpr>(tag_id, ident);
}

std::string Parser::parseIdentifier()
{
    skipWhitespace();

    size_t start = m_pos;

    while (m_pos < m_input.size()) {
        char c = m_input[m_pos];

        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
            ++m_pos;
        } else {
            break;
        }
    }

    return m_input.substr(start, m_pos - start);
}

bool Parser::match(char c)
{
    if (m_pos < m_input.size() && m_input[m_pos] == c) {
        ++m_pos;
        return true;
    }

    return false;
}

void Parser::skipWhitespace()
{
    while (m_pos < m_input.size() && std::isspace(static_cast<unsigned char>(m_input[m_pos]))) {
        ++m_pos;
    }
}

ExpressionPtr parse(const std::string& input, const std::map<std::string, int64_t>& tag_name_to_id)
{
    Parser parser(input, tag_name_to_id);
    return parser.parse();
}

} // namespace wmtk::components::simwild::expression_parser