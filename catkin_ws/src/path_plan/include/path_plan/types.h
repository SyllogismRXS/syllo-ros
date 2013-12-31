#ifndef SYLLO_TYPES_H_
#define SYLLO_TYPES_H_
/// ----------------------------------------------------------------------------
/// @file types.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-12-18 00:11:21 syllogismrxs>
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

#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace syllo
{

     typedef enum Cost{
          Zero     = 0,
          Straight = 10,
          Diagonal = 14
     } Cost_t;

     template <class T>
     class Point
     {
     public:
     Point() : x(-1), y(-1) { }
     Point(T xIn, T yIn) : x(xIn), y(yIn) { }
          T x;
          T y;

          
          double euclidean_distance(Point p)
          {
               double result;
               result = sqrt( pow((x-p.x),2) + pow(y-p.y,2) );
               return result;
          }
                    
          Point<T> operator+(const Point<T> &other)
          {
               return Point<T>(this->x + other.x, this->y + other.y);
          }
          
          Point<T> operator-(const Point<T> &other)
          {
               Point<T> point;
               point.x = this->x - other.x;
               point.y = this->y - other.y;
               return point;
          }
     
          Point<T> operator*(const double &other)
          {
               return Point<T>(this->x * other, this->y * other);
          }

          Point<T> operator*(const Point<T> &other)
          {
               return Point<T>(this->x * other.x, this->y * other.y);
          }

          Point<T> absolute()
          {
               return Point<T>(abs(this->x), abs(this->y));
          }

          

          //Point<double> operator+(const Point<int> &other)
          //{
          //     return Point<double>(this->x + (double)other.x, this->y + (double)other.x);
          //}

          //Point<double> operator-(const Point<int>

          bool operator==(const Point<T> &other) const
          {
               return (this->x == other.x) && (this->y == other.y);
          }
          
          bool operator!=(const Point<T> &other) const
          {
               return !(*this == other);
          }
          
          bool operator<(const Point<T> &rhs)
          {
               return (this->p1.x < rhs.x) || (this->p1.x == rhs.x && this->p1.y < rhs.y);
          }                    

     };        

     Point<double> add_points(const Point<double> &p1, const Point<int> &p2);
     Point<double> sub_points(const Point<int> &p1, const Point<double> &p2);
     //{
     //     Point<double> result;
     //     result.x = p1.x + (double)p2.x;
     //     result.y = p1.y + (double)p2.y;
     //     return result;
     //}

     //Point<double> operator+(const Point<double> &p1, const Point<int> &p2)
     //{
     //     return Point<double>(p1.x + (double)p2.x, p1.y + (double)p2.y);
     //}

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
          
          Point<int> point_;
          Cost_t cost_;
          Compass dir_;

          Point<int> point() { return point_; }
          Cost_t cost() { return cost_; }
          Compass dir() { return dir_; }

          Direction(Direction::Compass dir, Cost_t cost, Point<int> point) 
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
          Node(Point<int> point);
          Node(int x, int y);
          void reset();
          void compute_costs(Point<int> goal);
          double g() { return g_; }
          double h() { return h_; }
          double f() { return f_; }

          void set_list(List list) { list_ = list; }
          List list() { return list_; }
          
          Point<int> & point() { return point_; }

          //bool operator==(const Node &other) const;
          //bool operator!=(const Node &other) const;
          //friend bool operator<(const Node &n1, const Node &n2);
          
          bool operator<(const Node &other);

          void set_parent(Node *parent, Direction::Compass dir, Cost_t cost);
          Node * parent() { return parent_; }
          
     protected:
          Point<int> point_;

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
          int set_map(const std::vector<int> &map, int x_width, int y_height);
          int get_map(std::vector<int> &map, int &x_width, int &y_height);
          
          int set_map(const cv::Mat &map);
          int get_map(cv::Mat &map);

          int set_negated_map(const cv::Mat &map);
          int get_negated_map(cv::Mat &map);

          int at(int x, int y);
          void set_at(int value, int x, int y);
          
          int at(Point<int> point);
          void set_at(int value, Point<int> point);

          int x_width() { return x_width_; }
          int y_height() { return y_height_; }

          bool inMap(const Point<int> &point);
          

     protected:
          //Eigen::MatrixXi map_;
          typedef boost::multi_array<int, 2> array_type;
          typedef array_type::index index;
          array_type map_;
     
          //std::map<Point<int>, Node*> node_map_;
          
          int x_width_;
          int y_height_;
          Point<int> origin_;
          double resolution_; // (m / pixel)

     private:     
     
     };
}

#endif
