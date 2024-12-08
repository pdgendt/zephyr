from ply import lex

tokens = [
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
]

reserved = {
    '__attribute__': 'ATTRIBUTE',
    'struct': 'STRUCT',
}
tokens += reserved.values()

t_ATTRIBUTE = r'__attribute__'
t_ASSIGN = r'='
t_LEFTBRACE = r'{'
t_RIGHTBRACE = r'}'
t_LPAREN = r'\('
t_RPAREN = r'\)'
t_COMMA = r','
t_SEMICOLON = r';'
t_STRUCT = r'struct'


def t_ID(t):
    r'[a-zA-Z_][a-zA-Z0-9_]*'
    if t.value in reserved:
        t.type = reserved[t.value]
    return t


def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)


t_ignore = ' \t'


# Error handling rule
def t_error(t):
    print("Illegal character '%s'", t.value[0])


# Define tokens

# # Handle numbers
def t_NUMBER(t):
    r'\d+'
    t.value = int(t.value)
    return t


def t_STRING(t):
    r'"([^"\n]|(\\"))*"'
    return t


#
# # Ignore whitespace
# t_ignore = ' \t'
#
# # Track line numbers
# def t_newline(t):
#     r'\n+'
#     t.lexer.lineno += len(t.value)
#
# # Handle errors
# def t_error(t):
#     print(f"Illegal character '{t.value[0]}'")
#     t.lexer.skip(1)

# Build lexer
lexer = lex.lex()

# Example code with attributes
code = """
struct __attribute__((annotate("subsystem"))) my_driver_api {
uint8_t t1;
};
"""

lexer.input(code)
print("Tokens:")
for token in lexer:
    print(token)
