#ifndef A_STAR_H_
#define A_STAR_H_
/// ----------------------------------------------------------------------------
/// @file a_star.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-20 00:32:03 syllogismrxs>
///
/// @version 1.0
/// Created: 17 Sep 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The a_star class ...
/// 
/// ----------------------------------------------------------------------------

#include <iostream>
#include <list>
#include <map>
#include <vector>

#include "boost/multi_array.hpp"
#include <cassert>

#include <Eigen/Core>

#include "path_plan/types.h"

namespace syllo
{
     class AStar
     {
     private:
     protected:
          std::list<Node*> open_;
          std::list<Node*> closed_;

          Node start_;
          Node goal_;

          Map *map_;
          
          typedef boost::multi_array<Node*, 2> array_type;
          typedef array_type::index index;
          array_type node_map_;
     
          std::vector<Direction> directions_;

          std::list<Node*> path_;
          std::list<Node*> waypts_;

     public:    
          AStar();
          void reset();
          int set_map(Map *map);
          int generate_path(Node start, Node goal);
          std::list<Node*> & path();
          std::list<Node*> & waypts();
     };

}

#endif
