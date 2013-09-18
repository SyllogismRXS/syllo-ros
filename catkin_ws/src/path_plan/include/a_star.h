#ifndef A_STAR_H_
#define A_STAR_H_
/// ----------------------------------------------------------------------------
/// @file a_star.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-18 03:22:46 syllogismrxs>
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

#include <Eigen/Core>

namespace syllo
{

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

     class Node
     {     
     public:

          enum List {
               none = 0,
               closed,
               open
          };

          Node(Point point);
          Node(int x, int y);
          void compute_costs(Point goal);
          double g() { return g_; }
          double h() { return h_; }
          double f() { return f_; }

          void set_list(List list) { list_ = list; }
          List list() { return list_; }
          
          Point & point() { return point_; }

          bool operator==(const Node &other) const;
          bool operator!=(const Node &other) const;
          friend bool operator<(const Node &n1, const Node &n2);

     protected:
          Point point_;

          double g_;
          double h_;
          double f_;

          Node *parent_;

          List list_;
     private:     

     };

     class Map
     {
     public:
          enum Direction{
               N = 0,
               NE,
               E,
               SE,
               S,
               SW,
               W,
               NW
          };

          void set_origin(int x, int y);
          int fill_map(int *map, int rows, int cols);
          int at(int x, int y);
          void set_at(int value, int x, int y);
          
          int at(Point point);
          void set_at(int value, Point point);

     protected:
          Eigen::MatrixXi map_;
          
          std::map<Point, Node*> node_map_;

          Point origin_;
          double resolution_; // (m / pixel)

     private:     
     
     };
     
     class AStar
     {
     private:
     protected:
          std::list<Node*> open_;
          std::list<Node*> closed_;

          Node start_;
          Node goal_;

          Map *map_;
          
     public:                   
          int set_map(Map *map);
          int generate_path(Node start, Node goal);
     };

}

#endif
