from dataclasses import dataclass, field
from typing import Any

from ply import lex, yacc


tokens = [
    'IDENTIFIER', 'CONSTANT', 'STRING_LITERAL',
    'SIZEOF', 'PTR_OP', 'INC_OP', 'DEC_OP', 'LEFT_OP', 'RIGHT_OP',
    'LE_OP', 'GE_OP', 'EQ_OP', 'NE_OP', 'AND_OP', 'OR_OP', 'MUL_ASSIGN',
    'DIV_ASSIGN', 'MOD_ASSIGN', 'ADD_ASSIGN', 'SUB_ASSIGN', 'LEFT_ASSIGN',
    'RIGHT_ASSIGN', 'AND_ASSIGN', 'XOR_ASSIGN', 'OR_ASSIGN', 'TYPE_NAME',
    # Keywords
    'AUTO', 'BREAK', 'CASE', 'CHAR', 'CONST', 'CONTINUE', 'DEFAULT', 'DO',
    'DOUBLE', 'ELSE', 'ENUM', 'EXTERN', 'FLOAT', 'FOR', 'GOTO', 'IF', 'INLINE',
    'INT', 'LONG', 'REGISTER', 'RESTRICT', 'RETURN', 'SHORT', 'SIGNED', 'SIZEOF',
    'STATIC', 'STRUCT', 'SWITCH', 'TYPEDEF', 'UNION', 'UNSIGNED', 'VOID',
    'VOLATILE', 'WHILE', '_BOOL', '_COMPLEX', '_IMAGINARY'
]

states = (
    ('cattr', 'exclusive'),
    ('cargs', 'exclusive'),
)

literals = ['+', '-', '*', '/', '(', ')', '{', '}', ';', '=', ',', '.', '[', ']', ':', '<', '>']

t_INC_OP = r'\+\+'
t_DEC_OP = r'--'
t_PTR_OP = r'->'
t_LEFT_OP = r'<<'
t_RIGHT_OP = r'>>'
t_LE_OP = r'<='
t_GE_OP = r'>='
t_EQ_OP = r'=='
t_NE_OP = r'!='
t_AND_OP = r'&&'
t_OR_OP = r'\|\|'

def t_begin_cattr(t):
    r'\[\['
    t.lexer.cattr_start = t.lexer.lexpos
    t.lexer.push_state('cattr')


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


def t_ANY_IDENTIFIER(t):
    r'[a-zA-Z_][a-zA-Z_0-9]*'
    t.type = t.value.upper() if t.value.upper() in tokens else 'IDENTIFIER'
    return t


def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)


t_ANY_ignore = ' \t'


# Skip unknown
def t_ANY_error(t):
    t.lexer.skip(1)


# # Handle numbers
def t_ANY_CONSTANT(t):
    r'\d+'
    t.value = int(t.value)
    return t


def t_ANY_STRING(t):
    r'"([^"\n]|(\\"))*"'
    t.value = t.value[1:-1]
    return t


@dataclass(frozen=True)
class CAttr:
    id: str
    prefix: str = ""
    args: list[str | int] = field(default_factory=list)


@dataclass(frozen=True)
class StorageClassSpecifier:
    id: str


@dataclass(frozen=True)
class FunctionDefinition:
    # TODO: add all attributes
    declaration: str


@dataclass(frozen=True)
class Pointer:
    declarator: Any


# Backus-Naur ported from
# https://cs.wmich.edu/~gupta/teaching/cs4850/sumII06/The%20syntax%20of%20C%20in%20Backus-Naur%20form.htm
def p_translation_unit(p):
    '''
    translation-unit : external-declaration
                     | external-declaration translation-units
    '''
    if len(p) == 3:
        p[0] = p[1] + [p[2]]
    else:
        p[0] = [p[1]]

def p_translation_units(p):
    '''
    translation-units : empty
                      | translation-unit translation-units
    '''

def p_external_declaration(p):
    '''
    external-declaration : function-definition
                         | declaration
    '''
    p[0] = p[1]


def p_function_definition(p):
    '''
    function-definition : declaration-specifiers declarator declaration-list compound-statement
                        | declarator declaration-list compound-statement
    '''
    p[0] = FunctionDefinition(declaration=p[2])


def p_declaration(p):
    '''
    declaration : declaration-specifiers init-declarator-list ';'
                | init-declarator-list ';'
    '''


def p_init_declarator_list(p):
    '''
    init-declarator_list : init-declarator
                         | init-declarator_list ',' init-declarator
    '''

def p_init_declarator(p):
    '''
    init-declarator : declarator
                    | declarator '=' initializer
    '''


def p_declarator(p):
    '''
    declarator : pointer direct-declarator
               | direct-declarator
    '''
    if len(p) == 3:
        p[0] = Pointer(declarator=p[2])
    else:
        p[0] = p[1]


def p_direct_declarator(p):
    '''
    direct-declarator : IDENTIFIER
                      | '(' declarator ')'
                      | direct-declarator '[' constant-expression ']'
                      | direct-declarator '[' ']'
                      | direct-declarator '(' parameter-type-list ')'
                      | direct-declarator '(' identifier-list ')'
                      | direct-declarator '(' ')'
    '''


def p_initializer(p):
    '''
    initializer : assignment-expression
                | '{' initializer-list '}'
                | '{' initializer-list ',' '}'
    '''

def p_initializer_list(p):
    '''
    initializer-list : initializer
                     | initializer-list ',' initializer
    '''


def p_declaration_specifiers(p):
    '''
    declaration-specifiers : storage-class-specifier
                           | storage-class-specifier declaration-specifiers
                           | type-specifier
                           | type-specifier declaration-specifiers
                           | type-qualifier
                           | type-qualifier declaration-specifiers
    '''


def p_storage_class_specifier(p):
    '''
    storage-class-specifier : AUTO
                            | REGISTER
                            | STATIC
                            | EXTERN
                            | TYPEDEF
    '''
    p[0] = StorageClassSpecifier(id=p[1])


def p_type_specifier(p):
    '''
    type-specifier : VOID
                   | CHAR
                   | SHORT
                   | INT
                   | LONG
                   | FLOAT
                   | DOUBLE
                   | SIGNED
                   | UNSIGNED
                   | struct-or-union-specifier
                   | enum-specifier
                   | TYPE_NAME
    '''


def p_type_qualifier(p):
    '''
    type-qualifier : CONST
                   | VOLATILE
    '''


def p_struct_or_union_specifier(p):
    '''
    struct-or-union-specifier : struct-or-union IDENTIFIER '{' struct-declaration-list '}'
                              | struct-or-union '{' struct-declaration-list '}'
                              | struct-or-union IDENTIFIER
    '''


def p_struct_or_union(p):
    '''
    struct-or-union : STRUCT
                    | UNION
    '''


def p_struct_declaration_list(p):
    '''
    struct-declaration-list : struct-declaration
                            | struct-declaration-list struct-declaration
    '''


def p_struct_declaration(p):
    '''
    struct-declaration : specifier-qualifier-list struct-declarator-list ';'
    '''


def p_specifier_qualifier_list(p):
    '''
    specifier-qualifier-list : type-specifier specifier-qualifier-list
                             | type-specifier
                             | type-qualifier specifier-qualifier-list
                             | type-qualifier
    '''


def p_struct_declarator_list(p):
    '''
    struct-declarator-list : struct-declarator
                           | struct-declarator-list ',' struct-declarator
    '''

def p_struct_declarator(p):
    '''
    struct-declarator : declarator
                      | ':' constant-expression
                      | declarator ':' constant-expression
    '''

def p_enum_specifier(p):
    '''
    enum-specifier : ENUM IDENTIFIER '{' enumerator-list '}'
                   | ENUM '{' enumerator-list '}'
                   | ENUM IDENTIFIER
    '''

def p_enumerator_list(p):
    '''
    enumerator-list : enumerator
                    | enumerator-list ',' enumerator
    '''

def p_enumerator(p):
    '''
    enumerator : IDENTIFIER
               | IDENTIFIER '=' constant-expression
    '''

def p_expression(p):
    '''
    expression : assignment-expression
               | expression ',' assignment-expression
    '''

def p_assignment_expression(p):
    '''
    assignment-expression : conditional-expression
                          | unary-expression assignment-operator assignment-expression
    '''

def p_assignment_operator(p):
    '''
    assignment-operator : '='
                        | MUL_ASSIGN
                        | DIV_ASSIGN
                        | MOD_ASSIGN
                        | ADD_ASSIGN
                        | SUB_ASSIGN
                        | LEFT_ASSIGN
                        | RIGHT_ASSIGN
                        | AND_ASSIGN
                        | XOR_ASSIGN
                        | OR_ASSIGN
    '''

def p_conditional_expression(p):
    '''
    conditional-expression : logical-or-expression
                           | logical-or-expression '?' expression ':' conditional-expression
    '''

def p_logical_or_expression(p):
    '''
    logical-or-expression : logical-and-expression
                          | logical-or-expression OR_OP logical-and-expression
    '''

def p_logical_and_expression(p):
    '''
    logical_and_expression : inclusive_or_expression
                           | logical_and_expression AND_OP inclusive_or_expression
    '''

def p_statement(p):
    '''
    statement : labeled-statement
              | compound-statement
              | expression-statement
              | selection-statement
              | iteration-statement
              | jump-statement
    '''

def p_labeled_statement(p):
    '''
    labeled-statement : IDENTIFIER ':' statement
                      | CASE constant-expression ':' statement
                      | DEFAULT ':' statement
    '''

def p_compound_statement(p):
    '''
    compound_statement : '{' '}'
                       | '{' block_item_list '}'
    '''

def p_block_item_list(p):
    '''
    block-item-list : block-item
                    | block-item-list block-item
    '''

def p_block_item(p):
    '''
    block_item : declaration
               | statement
    '''

def p_expression_statement(p):
    '''
    expression_statement : ';'
                         | expression ';'
    '''

def p_selection_statement(p):
    '''
    selection_statement : IF '(' expression ')' statement
                        | IF '(' expression ')' statement ELSE statement
                        | SWITCH '(' expression ')' statement
    '''

def p_iteration_statement(p):
    '''
    iteration_statement : WHILE '(' expression ')' statement
                        | DO statement WHILE '(' expression ')' ';'
                        | FOR '(' expression_statement expression_statement ')' statement
                        | FOR '(' expression_statement expression_statement expression ')' statement
    '''

def p_jump_statement(p):
    '''
    jump_statement : GOTO IDENTIFIER ';'
                   | CONTINUE ';'
                   | BREAK ';'
                   | RETURN ';'
                   | RETURN expression ';'
    '''
##########################################################################


def p_cattr_declaration(p):
    '''
    cattr-declaration : cattr-standard
                      | cattr-prefix
                      | cattr-cargs
                      | cattr-prefix_cargs
    '''
    p[0] = p[1]


def p_cattr_standard(p):
    '''
    cattr-standard : ID CATTR
    '''
    p[0] = CAttr(id=p[1])


def p_cattr_prefix(p):
    '''
    cattr-prefix : ID DCOLON ID CATTR
    '''
    p[0] = CAttr(id=p[3], prefix=p[1])


def p_cattr_cargs(p):
    '''
    cattr-cargs : ID cargs_list CATTR
    '''
    p[0] = CAttr(id=p[1], args=p[2])


def p_cattr_prefix_cargs(p):
    '''
    cattr-prefix_cargs : ID DCOLON ID cargs_list CARGS CATTR
    '''
    p[0] = CAttr(id=p[3], prefix=p[1], args=p[4])


def p_cargs_list(p):
    '''
    cargs_list : cargs_val
               | cargs_list ',' cargs_val
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


def p_lists(p):
    '''
    declaration-specifier-star : declaration-specifier-star declaration-specifier
                               | empty
    declaration-star : declaration-star declaration
                     | empty
    '''
    if len(p) > 2:
        p[0] = p[1] + [p[2]]
    else:
        p[0] = [p[1]]


def p_empty(_):
    'empty :'
    pass


def p_error(p):
    print('error', p)


def main():
    # Example code with attributes
    # code = """
    # void [[zpp::syscall]] my_func(const struct device *dev, uint8_t arg);
    # struct [[zpp::annotate("subsystem","test")]] my_driver_api {};
    # """
    code = """
    [[zpp::syscall]]
    [[zpp::annotate("subsystem", "test")]]
    """

    lexer = lex.lex()
    parser = yacc.yacc()
    print(parser.parse(code, lexer=lexer))


if __name__ == "__main__":
    main()
