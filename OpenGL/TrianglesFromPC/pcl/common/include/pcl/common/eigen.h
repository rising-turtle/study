/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: eigen.h 1774 2011-07-14 23:03:22Z rusu $
 *
 */
// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

// The computeRoots function included in this is based on materials
// covered by the following copyright and license:
// 
// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// 
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:
// 
// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef PCL_EIGEN_H_
#define PCL_EIGEN_H_

#define NOMINMAX

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace pcl
{
	template<typename Scalar, typename Roots> inline void computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots)
	{
		roots(0) = Scalar(0);
		Scalar d = b * b - 4.0 * c;
		if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
			d = 0.0;

		Scalar sd = sqrt (d);
		
		roots (2) = 0.5f * (b + sd);
		roots (1) = 0.5f * (b - sd);
	}
	
  template<typename Matrix, typename Roots> inline void 
  computeRoots (const Matrix& m, Roots& roots)
  {
    typedef typename Matrix::Scalar Scalar;

    // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
    // eigenvalues are the roots to this equation, all guaranteed to be
    // real-valued, because the matrix is symmetric.
    Scalar c0 =               m(0,0)*m(1,1)*m(2,2) 
                + Scalar(2) * m(0,1)*m(0,2)*m(1,2) 
                            - m(0,0)*m(1,2)*m(1,2) 
                            - m(1,1)*m(0,2)*m(0,2) 
                            - m(2,2)*m(0,1)*m(0,1);
    Scalar c1 = m(0,0)*m(1,1) - 
                m(0,1)*m(0,1) + 
                m(0,0)*m(2,2) - 
                m(0,2)*m(0,2) + 
                m(1,1)*m(2,2) - 
                m(1,2)*m(1,2);
    Scalar c2 = m(0,0) + m(1,1) + m(2,2);


		if (fabs(c0) < Eigen::NumTraits<Scalar>::epsilon())// one root is 0 -> quadratic equation
			computeRoots2 (c2, c1, roots);
		else
		{
		  const Scalar s_inv3 = 1.0/3.0;
		  const Scalar s_sqrt3 = Eigen::internal::sqrt (Scalar (3.0));
		  // Construct the parameters used in classifying the roots of the equation
		  // and in solving the equation for the roots in closed form.
		  Scalar c2_over_3 = c2*s_inv3;
		  Scalar a_over_3 = (c1 - c2*c2_over_3)*s_inv3;
		  if (a_over_3 > Scalar (0))
		    a_over_3 = Scalar (0);

		  Scalar half_b = Scalar(0.5) * (c0 + c2_over_3 * (Scalar(2) * c2_over_3 * c2_over_3 - c1));

		  Scalar q = half_b*half_b + a_over_3*a_over_3*a_over_3;
		  if (q > Scalar(0))
		    q = Scalar(0);

		  // Compute the eigenvalues by solving for the roots of the polynomial.
		  Scalar rho = Eigen::internal::sqrt (-a_over_3);
		  Scalar theta = std::atan2 (Eigen::internal::sqrt (-q), half_b)*s_inv3;
		  Scalar cos_theta = Eigen::internal::cos (theta);
		  Scalar sin_theta = Eigen::internal::sin (theta);
		  roots(0) = c2_over_3 + Scalar(2) * rho * cos_theta;
		  roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
		  roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

		  // Sort in increasing order.
		  if (roots (0) >= roots (1))
		    std::swap (roots (0), roots (1));
		  if (roots (1) >= roots (2))
		  {
		    std::swap (roots (1), roots (2));
		    if (roots (0) >= roots (1))
		      std::swap (roots (0), roots (1));
		  }
		  
		  if (roots(0) <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
			  computeRoots2 (c2, c1, roots);
		}
  }

  template<typename Matrix, typename Vector> void 
  eigen33 (const Matrix& mat, Matrix& evecs, Vector& evals)
  {
    typedef typename Matrix::Scalar Scalar;
    // Scale the matrix so its entries are in [-1,1].  The scaling is applied
    // only when at least one matrix entry has magnitude larger than 1.

    Scalar scale = mat.cwiseAbs ().maxCoeff ();
    if (scale <= std::numeric_limits<Scalar>::min())
    	scale = Scalar(1.0);
    
    Matrix scaledMat = mat / scale;

    // Compute the eigenvalues
    computeRoots (scaledMat,evals);
    
		if((evals(2)-evals(0))<=Eigen::NumTraits<Scalar>::epsilon())
		{
			// all three equal
			evecs.setIdentity();
		}
		else if ((evals(1)-evals(0))<=Eigen::NumTraits<Scalar>::epsilon() )
		{
			// first and second equal
			Matrix tmp;
			tmp = scaledMat;
			tmp.diagonal ().array () -= evals (2);

			Vector vec1 = tmp.row (0).cross (tmp.row (1));
			Vector vec2 = tmp.row (0).cross (tmp.row (2));
			Vector vec3 = tmp.row (1).cross (tmp.row (2));

			Scalar len1 = vec1.squaredNorm();
			Scalar len2 = vec2.squaredNorm();
			Scalar len3 = vec3.squaredNorm();

			if (len1 >= len2 && len1 >= len3)
			 	evecs.col (2) = vec1 / Eigen::internal::sqrt (len1);
			else if (len2 >= len1 && len2 >= len3)
		 		evecs.col (2) = vec2 / Eigen::internal::sqrt (len2);
			else
				evecs.col (2) = vec3 / Eigen::internal::sqrt (len3);
		
			evecs.col (1) = evecs.col (2).unitOrthogonal ();
			evecs.col (0) = evecs.col (1).cross (evecs.col (2));
		}
		else if ((evals(2)-evals(1))<=Eigen::NumTraits<Scalar>::epsilon() )
		{
			// second and third equal
			Matrix tmp;
			tmp = scaledMat;
			tmp.diagonal ().array () -= evals (0);

			Vector vec1 = tmp.row (0).cross (tmp.row (1));
			Vector vec2 = tmp.row (0).cross (tmp.row (2));
			Vector vec3 = tmp.row (1).cross (tmp.row (2));

			Scalar len1 = vec1.squaredNorm();
			Scalar len2 = vec2.squaredNorm();
			Scalar len3 = vec3.squaredNorm();

			if (len1 >= len2 && len1 >= len3)
			 	evecs.col (0) = vec1 / Eigen::internal::sqrt (len1);
			else if (len2 >= len1 && len2 >= len3)
		 		evecs.col (0) = vec2 / Eigen::internal::sqrt (len2);
			else
				evecs.col (0) = vec3 / Eigen::internal::sqrt (len3);

			evecs.col (1) = evecs.col (0).unitOrthogonal ();
			evecs.col (2) = evecs.col (0).cross (evecs.col (1));
		}
		else
		{
		  Matrix tmp;
		  tmp = scaledMat;
		  tmp.diagonal ().array () -= evals (2);

		  Vector vec1 = tmp.row (0).cross (tmp.row (1));
		  Vector vec2 = tmp.row (0).cross (tmp.row (2));
		  Vector vec3 = tmp.row (1).cross (tmp.row (2));

		  Scalar len1 = vec1.squaredNorm ();
		  Scalar len2 = vec2.squaredNorm ();
		  Scalar len3 = vec3.squaredNorm ();
#ifdef _WIN32
		  Scalar *mmax = new Scalar[3];
#else
			Scalar mmax[3];
#endif
		  unsigned int min_el = 2;
		  unsigned int max_el = 2;
		  if (len1 >= len2 && len1 >= len3)
		  {
		    mmax[2] = len1;
		    evecs.col (2) = vec1 / Eigen::internal::sqrt (len1);
		  }
		  else if (len2 >= len1 && len2 >= len3)
		  {
		    mmax[2] = len2;
		    evecs.col (2) = vec2 / Eigen::internal::sqrt (len2);
		  }
		  else
		  {
		    mmax[2] = len3;
		    evecs.col (2) = vec3 / Eigen::internal::sqrt (len3);
		  }

		  tmp = scaledMat;
		  tmp.diagonal ().array () -= evals (1);

		  vec1 = tmp.row (0).cross (tmp.row (1));
		  vec2 = tmp.row (0).cross (tmp.row (2));
		  vec3 = tmp.row (1).cross (tmp.row (2));

		  len1 = vec1.squaredNorm ();
		  len2 = vec2.squaredNorm ();
		  len3 = vec3.squaredNorm ();
		  if (len1 >= len2 && len1 >= len3)
		  {
		    mmax[1] = len1;
		    evecs.col (1) = vec1 / Eigen::internal::sqrt (len1);
		    min_el = len1 <= mmax[min_el]? 1: min_el;
		    max_el = len1 > mmax[max_el]? 1: max_el;
		  }
		  else if (len2 >= len1 && len2 >= len3)
		  {
		    mmax[1] = len2;
		    evecs.col (1) = vec2 / Eigen::internal::sqrt (len2);
		    min_el = len2 <= mmax[min_el]? 1: min_el;
		    max_el = len2 > mmax[max_el]? 1: max_el;
		  }
		  else
		  {
		    mmax[1] = len3;
		    evecs.col (1) = vec3 / Eigen::internal::sqrt (len3);
		    min_el = len3 <= mmax[min_el]? 1: min_el;
		    max_el = len3 > mmax[max_el]? 1: max_el;
		  }
		  
		  tmp = scaledMat;
		  tmp.diagonal ().array () -= evals (0);

		  vec1 = tmp.row (0).cross (tmp.row (1));
		  vec2 = tmp.row (0).cross (tmp.row (2));
		  vec3 = tmp.row (1).cross (tmp.row (2));

		  len1 = vec1.squaredNorm ();
		  len2 = vec2.squaredNorm ();
		  len3 = vec3.squaredNorm ();
		  if (len1 >= len2 && len1 >= len3)
		  {
		    mmax[0] = len1;
		    evecs.col (0) = vec1 / Eigen::internal::sqrt (len1);
		    min_el = len3 <= mmax[min_el]? 0: min_el;
		    max_el = len3 > mmax[max_el]? 0: max_el;
		  }
		  else if (len2 >= len1 && len2 >= len3)
		  {
		    mmax[0] = len2;
		    evecs.col (0) = vec2 / Eigen::internal::sqrt (len2);
		    min_el = len3 <= mmax[min_el]? 0: min_el;
		    max_el = len3 > mmax[max_el]? 0: max_el; 		
		  }
		  else
		  {
		    mmax[0] = len3;
		    evecs.col (0) = vec3 / Eigen::internal::sqrt (len3);
		    min_el = len3 <= mmax[min_el]? 0: min_el;
		    max_el = len3 > mmax[max_el]? 0: max_el;	  
		  }
		  
		  unsigned mid_el = 3 - min_el - max_el;
		  evecs.col (min_el) = evecs.col ((min_el+1)%3).cross ( evecs.col ((min_el+2)%3) ).normalized ();
			evecs.col (mid_el) = evecs.col ((mid_el+1)%3).cross ( evecs.col ((mid_el+2)%3) ).normalized ();
#ifdef _WIN32		
		  delete [] mmax;
#endif
		}
	  // Rescale back to the original size.
	  evals *= scale;
  }
}

#endif  //#ifndef PCL_EIGEN_H_