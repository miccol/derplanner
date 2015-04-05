//
// Copyright (c) 2015 Alexander Shafranov shafranov@gmail.com
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
// claim that you wrote the original software. If you use this software
// in a product, an acknowledgment in the product documentation would be
// appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DERPLANNER_COMPILER_PARSER_H_
#define DERPLANNER_COMPILER_PARSER_H_

#include "derplanner/compiler/types.h"

namespace plnnrc {

/// Parser

// create parsing state.
void init(Parser& state, Lexer* lexer);

// destroy parsing state.
void destroy(Parser& state);

// parse token stream to abstract-syntax-tree.
void parse(Parser& state);

/// Expr

// make node `child` the last child of node `parent`.
void append_child(ast::Expr* parent, ast::Expr* child);

// make `child` the next sibling of `after`.
void insert_child(ast::Expr* after, ast::Expr* child);

// unparent `node` from it's current parent.
void unparent(ast::Expr* node);

// returns the next node in preorder (visit node then visit it's children) traversal.
ast::Expr* preorder_next(const ast::Expr* root, ast::Expr* current);

}

#endif