/* AUTORIGHTS
   Copyright (c) 2007 The Regents of the University of California.
   All Rights Reserved.
 
   Created by Stefano Carpin
   University of California, Merced, Robotics Lab - School of Engineering
 
   Permission to use, copy, modify, and distribute this software and its
   documentation for educational, research and non-profit purposes,
   without fee, and without a written agreement is hereby granted,
   provided that the above copyright notice, this paragraph and the
   following three paragraphs appear in all copies.
 
   This software program and documentation are copyrighted by The Regents
   of the University of California. The software program and
   documentation are supplied "as is", without any accompanying services
   from The Regents. The Regents does not warrant that the operation of
   the program will be uninterrupted or error-free. The end-user
   understands that the program was developed for research purposes and
   is advised not to rely exclusively on the program for any reason.
 
   IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY
   FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
   INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND
   ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE UNIVERSITY OF
   CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
   BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATIONS TO PROVIDE
   MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

*/


#include "maps_hough/grid_map.hpp"
#include "maps_hough/io.hpp"
#include "maps_hough/common.hpp"
#include "maps_hough/hough.hpp"
#include "maps_hough/manipulatemap.hpp"

#include <iterator>
#include <iostream>
#include <algorithm>

#include <sys/time.h>

#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace mapmerge;

const unsigned int n_hypothesis = 4;

int main()
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("maps_hough");
  std::string file1 = package_share_directory + "/datasets/LongRun6.txt";
  std::string file2 = package_share_directory + "/datasets/LongRun7.txt";
    
  std::cout << "Loading " << file1 << " and " << file2 << std::endl;
  grid_map a, b;
  a.load_map(900, 900, file1);
  b.load_map(900, 900, file2);
	   
  
  std::vector<transformation> hyp = get_hypothesis(a,b,n_hypothesis,1,false);
    
  for ( unsigned int i = 0 ; i < n_hypothesis ; i++ )
    std::cout << hyp[i].ai << " " << hyp[i].deltax << " " << hyp[i].deltay << " " <<  hyp[i].rotation << std::endl;
    

  return 0;
}
