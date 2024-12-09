from ply import lex, yacc

tokens = [
    'CATTR',
    'CARGS',
    'ASSIGN',
    'NUMBER',
    'STRING',
    'LPAREN',
    'RPAREN',
    'ID',
    'COMMA',
    'SEMICOLON',
    'LEFTBRACE',
    'RIGHTBRACE',
    'DCOLON',
]

reserved = {
    'struct': 'STRUCT',
    'union': 'UNION',
}

states = (
    ('cattr', 'exclusive'),
    ('cargs', 'exclusive'),
)

tokens += reserved.values()

t_ASSIGN = r'='
t_LEFTBRACE = r'{'
t_RIGHTBRACE = r'}'
t_LPAREN = r'\('
t_RPAREN = r'\)'
t_ANY_COMMA = r','
t_ANY_SEMICOLON = r';'
t_STRUCT = r'struct'
t_UNION = r'union'


def t_begin_cattr(t):
    r'\[\['
    t.lexer.cattr_start = t.lexer.lexpos
    t.lexer.push_state('cattr')


t_cattr_DCOLON = r'::'


def t_cattr_begin_cargs(t):
    r'\('
    t.lexer.cargs_start = t.lexer.lexpos
    t.lexer.push_state('cargs')


def t_cattr_cargs_end(t):
    r'\)'
    t.value = t.lexer.lexdata[t.lexer.cargs_start : t.lexer.lexpos - 1]
    t.type = "CARGS"
    t.lexer.pop_state()
    return t


def t_cattr_end(t):
    r'\]\]'
    t.value = t.lexer.lexdata[t.lexer.cattr_start : t.lexer.lexpos - 2]
    t.type = "CATTR"
    t.lexer.pop_state()
    return t


def t_ANY_ID(t):
    r'[a-zA-Z_][a-zA-Z0-9_]*'
    if t.value in reserved:
        t.type = reserved[t.value]
    return t


def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)


t_ANY_ignore = ' \t'


# Skip unknown
def t_ANY_error(t):
    t.lexer.skip(1)


# # Handle numbers
def t_ANY_NUMBER(t):
    r'\d+'
    t.value = int(t.value)
    return t


def t_ANY_STRING(t):
    r'"([^"\n]|(\\"))*"'
    return t


# Build lexer
lexer = lex.lex()

# Example code with attributes
code = """
struct [[zpp::annotate("subsystem","test")]] my_driver_api {
    uint8_t t1;
};
"""

lexer.input(code)
print("Tokens:")
for token in lexer:
    print(token)
