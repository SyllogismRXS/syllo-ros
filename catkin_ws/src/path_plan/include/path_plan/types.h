#ifndef TYPES_H_
#define TYPES_H_
/// ----------------------------------------------------------------------------
/// @file types.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-18 20:51:43 syllogismrxs>
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
/// The types classes ...
/// 
/// ----------------------------------------------------------------------------

#include <iostream>
#include <list>
#include <map>
#include <vector>

#include "boost/multi_array.hpp"
#include <cassert>

namespace syllo
{

     typedef enum Cost{
          Zero     = 0,
          Straight = 10,
          Diagonal = 14
     } Cost_t;

     class Point
     {
     public:
     Point() : x(-1), y(-1) { }
     Point(int xIn, int yIn) : x(xIn), y(yIn) { }
          int x;
          int y;

          double euclidean_distance(Point p);

          friend Point operator+(const Point &p1, const Point &p2);

          bool operator==(const Point &other) const;
          bool operator!=(const Point &other) const;
          friend bool operator<(const Point &p1, const Point &p2);
     };

     class Direction {
     public:
          enum Compass{
               N = 0,
               NE,
               E,
               SE,
               S,
               SW,
               W,
               NW
          };
          
          Point point_;
          Cost_t cost_;
          Compass dir_;

          Point point() { return point_; }
          Cost_t cost() { return cost_; }
          Compass dir() { return dir_; }

          Direction(Direction::Compass dir, Cost_t cost, Point point) 
          {
               dir_ = dir;
               cost_ = cost;
               point_ = point;
          }
     };

     class Node
     {     
     public:

          enum List {
               None = 0,
               Closed,
               Open
          };

          Node();
          Node(Point point);
          Node(int x, int y);
          void reset();
          void compute_costs(Point goal);
          double g() { return g_; }
          double h() { return h_; }
          double f() { return f_; }

          void set_list(List list) { list_ = list; }
          List list() { return list_; }
          
          Point & point() { return point_; }

          //bool operator==(const Node &other) const;
          //bool operator!=(const Node &other) const;
          //friend bool operator<(const Node &n1, const Node &n2);
          
          bool operator<(const Node &other);

          void set_parent(Node *parent, Direction::Compass dir, Cost_t cost);
          Node * parent() { return parent_; }
          
     protected:
          Point point_;

          double g_;
          double h_;
          double f_;

          Node *parent_;
          Direction::Compass direction_from_parent_;
          Cost_t cost_from_parent_;

          List list_;
     private:     

     };

     class Map
     {
     public:          
          void set_origin(int x, int y);
          int fill_map(const std::vector<int> &map, int x_width, int y_height);
          int at(int x, int y);
          void set_at(int value, int x, int y);
          
          int at(Point point);
          void set_at(int value, Point point);

          int x_width() { return x_width_; }
          int y_height() { return y_height_; }

          bool inMap(const Point &point);
          

     protected:
          //Eigen::MatrixXi map_;
          typedef boost::multi_array<int, 2> array_type;
          typedef array_type::index index;
          array_type map_;
     
          //std::map<Point, Node*> node_map_;
          
          int x_width_;
          int y_height_;
          Point origin_;
          double resolution_; // (m / pixel)

     private:     
     
     };
}

#endif
