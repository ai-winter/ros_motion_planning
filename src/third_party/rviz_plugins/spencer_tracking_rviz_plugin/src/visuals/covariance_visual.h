/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
 *  Copyright (c) 2006, Kai O. Arras, Autonomous Intelligent Systems Lab, University of Freiburg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COVARIANCE_VISUAL_H
#define COVARIANCE_VISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <cmath>

namespace spencer_tracking_rviz_plugin
{
// Visualization of a covariance matrix
class CovarianceVisual
{
public:
  CovarianceVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode) : m_sceneManager(sceneManager)
  {
    m_sceneNode = parentNode->createChildSceneNode();
  }

  virtual ~CovarianceVisual()
  {
    m_sceneManager->destroySceneNode(m_sceneNode->getName());
  };

  void setPosition(const Ogre::Vector3& position)
  {
    m_sceneNode->setPosition(position);
  }

  void setOrientation(const Ogre::Quaternion& orientation)
  {
    m_sceneNode->setOrientation(orientation);
  }

  void setVisible(bool visible)
  {
    m_sceneNode->setVisible(visible, true);
  }

  virtual void setColor(const Ogre::ColourValue& c) = 0;

  virtual void setLineWidth(float lineWidth) = 0;

  /// NOTE: It is assumed that the covariance matrix is already rotated into the target frame of the sceneNode!
  virtual void setMeanCovariance(const Ogre::Vector3& mean, const Ogre::Matrix3& cov) = 0;

protected:
  Ogre::SceneManager* m_sceneManager;
  Ogre::SceneNode* m_sceneNode;
};

// 2D ellipse visualization of a covariance matrix
class ProbabilityEllipseCovarianceVisual : public CovarianceVisual
{
public:
  ProbabilityEllipseCovarianceVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode)
    : CovarianceVisual(sceneManager, parentNode)
  {
    m_line = new rviz::BillboardLine(m_sceneManager, m_sceneNode);
  }

  virtual ~ProbabilityEllipseCovarianceVisual()
  {
    delete m_line;
  }

  virtual void setLineWidth(float lineWidth)
  {
    m_line->setLineWidth(lineWidth);
  }

  virtual void setColor(const Ogre::ColourValue& c)
  {
    m_line->setColor(c.r, c.g, c.b, c.a);
  }

  virtual void setMeanCovariance(const Ogre::Vector3& mean, const Ogre::Matrix3& cov)
  {
    int numberOfPoints;
    double *xs, *ys;
    double determinant = cov[0][0] * cov[1][1] - cov[1][0] * cov[0][1];

    m_line->clear();

    if (!std::isfinite(determinant))
    {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "Covariance matrix has non-finite values in ProbabilityEllipseCovarianceVisual::setMeanCovariance(): "
                   << cov);
      return;
    }

    if (std::abs(cov[0][1] - cov[1][0]) > 0.00001)
    {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "Covariance matrix is not symmetric in ProbabilityEllipseCovarianceVisual::setMeanCovariance(): " << cov);
      return;
    }

    if (determinant > 0 && cov[0][0] > 0 /* positive definite? */ ||
        std::abs(determinant - 0.00001) == 0.0 && (cov[0][0] > 0 || cov[1][1] > 0) /* positive semidefinite? */)
    {
      calc_prob_elli_99(mean.x, mean.y, cov[0][0], cov[1][1], cov[0][1], numberOfPoints, xs, ys);

      m_line->setMaxPointsPerLine(numberOfPoints);

      for (int i = 0; i < numberOfPoints; i++)
      {
        Ogre::Vector3 vertex(xs[i], ys[i], mean.z);
        m_line->addPoint(vertex);
      }
    }
    else
    {
      if (cov[0][0] != 0 || cov[0][1] != 0 || cov[0][2] != 0 || cov[1][0] != 0 || cov[1][1] != 0 || cov[1][2] != 0 ||
          cov[2][0] != 0 || cov[2][1] != 0 || cov[2][2] != 0)
        ROS_WARN_STREAM_THROTTLE(5.0,
                                 "Covariance matrix is not positive (semi-)definite in "
                                 "ProbabilityEllipseCovarianceVisual::setMeanCovariance(): "
                                     << cov);
    }
  }

private:
  rviz::BillboardLine* m_line;

  // Puts angle alpha into the interval [min..min+2*pi[
  double set_angle_to_range(double alpha, double min)
  {
    while (alpha >= min + 2.0 * M_PI)
    {
      alpha -= 2.0 * M_PI;
    }
    while (alpha < min)
    {
      alpha += 2.0 * M_PI;
    }
    return alpha;
  }

  // Calculates the points on a rotated ellipse given by center xc, yc, half axes a, b and angle phi.
  // Returns number of points np and points in Cart. coordinates
  void calc_ellipse(double xc, double yc, double a, double b, double phi, int& np, double*& xvec, double*& yvec)
  {
    const int N_ELLI_POINTS = 40;
    int i, offset;
    double t, cr, sr, ca, sa, xi, yi, reso;
    static double x[N_ELLI_POINTS + 1];
    static double y[N_ELLI_POINTS + 1];
    reso = 2 * M_PI / N_ELLI_POINTS;
    offset = N_ELLI_POINTS / 2;
    ca = cos(phi);
    sa = sin(phi);
    i = 0;
    t = 0;
    while (t < M_PI)
    {
      cr = cos(t);
      sr = sin(t);
      xi = a * cr * ca - b * sr * sa;
      yi = a * cr * sa + b * sr * ca;
      x[i] = xi + xc;
      y[i] = yi + yc;
      x[offset + i] = -xi + xc;
      y[offset + i] = -yi + yc;
      t = t + reso;
      i++;
    }
    x[N_ELLI_POINTS] = x[0];  // Close contour
    y[N_ELLI_POINTS] = y[0];  // Close contour
    np = N_ELLI_POINTS + 1;
    xvec = x;
    yvec = y;
  }

  // Calculates the points on a 95%-iso-probability ellipse given by the bivarate RV with mean xc, yc
  // and covariance matrix sxx, syy, sxy. Returns number of points np and points in Cart. coordinates
  void calc_prob_elli_95(double xc, double yc, double sxx, double syy, double sxy, int& np, double*& x, double*& y)
  {
    double la, lb, a, b, phi;
    la = (sxx + syy + sqrt((sxx - syy) * (sxx - syy) + 4 * sxy * sxy)) / 2;
    lb = (sxx + syy - sqrt((sxx - syy) * (sxx - syy) + 4 * sxy * sxy)) / 2;
    a = sqrt(5.991464 * la);
    b = sqrt(5.991464 * lb);
    phi = set_angle_to_range(atan2(2 * sxy, sxx - syy) / 2, 0);
    calc_ellipse(xc, yc, a, b, phi, np, x, y);
  }

  // Calculates the points on a 99%-iso-probability ellipse given by the bivarate RV with mean xc, yc
  // and covariance matrix sxx, syy, sxy. Returns number of points np and points in Cart. coordinates
  void calc_prob_elli_99(double xc, double yc, double sxx, double syy, double sxy, int& np, double*& x, double*& y)
  {
    double la, lb, a, b, phi;
    la = (sxx + syy + sqrt((sxx - syy) * (sxx - syy) + 4 * sxy * sxy)) / 2;
    lb = (sxx + syy - sqrt((sxx - syy) * (sxx - syy) + 4 * sxy * sxy)) / 2;
    a = sqrt(9.210340 * la);
    b = sqrt(9.210340 * lb);
    phi = set_angle_to_range(atan2(2 * sxy, sxx - syy) / 2, 0);
    calc_ellipse(xc, yc, a, b, phi, np, x, y);
  }
};

}  // namespace spencer_tracking_rviz_plugin

#endif  // COVARIANCE_VISUAL_H
