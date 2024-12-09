from dataclasses import dataclass, field

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
    t.value = t.value[1:-1]
    return t


@dataclass()
class CAttr:
    id: str
    prefix: str = ""
    args: list[str | int] = field(default_factory=list)


def p_cattr_declaration(p):
    '''
    cattr_delcaration : cattr_standard
                      | cattr_prefix
                      | cattr_cargs
                      | cattr_prefix_cargs
    '''
    p[0] = p[1]


def p_cattr_standard(p):
    '''
    cattr_standard : ID CATTR
    '''
    p[0] = CAttr(id=p[1])


def p_cattr_prefix(p):
    '''
    cattr_prefix : ID DCOLON ID CATTR
    '''
    p[0] = CAttr(id=p[3], prefix=p[1])


def p_cattr_cargs(p):
    '''
    cattr_cargs : ID cargs_list CATTR
    '''
    p[0] = CAttr(id=p[1], args=p[2])


def p_cattr_prefix_cargs(p):
    '''
    cattr_prefix_cargs : ID DCOLON ID cargs_list CARGS CATTR
    '''
    p[0] = CAttr(id=p[3], prefix=p[1], args=p[4])


def p_cargs_list(p):
    '''
    cargs_list : cargs_val
               | cargs_list COMMA cargs_val
    '''
    if len(p) == 2:
        p[0] = [p[1]]
    else:
        p[0] = p[1] + [p[3]]


def p_cargs_val(p):
    '''
    cargs_val : STRING
              | NUMBER
              | empty
    '''
    p[0] = p[1]


def p_empty(_):
    'empty :'
    pass


def p_error(p):
    print('error', p)


def main():
    # Example code with attributes
    code = """
    void [[zpp::syscall]] my_func(const struct device *dev, uint8_t arg);
    struct [[zpp::annotate("subsystem","test")]] my_driver_api {};
    """

    lexer = lex.lex()
    parser = yacc.yacc()
    print(parser.parse(code, lexer=lexer))


if __name__ == "__main__":
    main()
