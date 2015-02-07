//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#include "ped_vector.h"

#include <cmath>
#include <string>
#include <sstream>
/// Default constructor, which makes sure that all the values are set to 0.
/// \date    2012-01-16
Ped::Tvector::Tvector() {
  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0;
};


std::string Ped::Tvector::to_string() const {

  std::ostringstream strs;
  strs << pos[0] << "/" << pos[1] << "/" << pos[2];
  return strs.str();
  //return std::to_string((const long double) x) + "/" + std::to_string((const long double) y_) + "/" + std::to_string((const long double) z_);
}


/// Returns the length of the vector.
/// \return the length
double Ped::Tvector::length() const {
    if ((pos[0] == 0) && (pos[1] == 0) && (pos[2] == 0)) return 0;
    return sqrt(lengthSquared());
}


/// Returns the length of the vector squared. This is faster than the real length.
/// \return the length squared
double Ped::Tvector::lengthSquared() const {
  return pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2];
}


/// Normalizes the vector to a length of 1.
/// \date    2010-02-12
void Ped::Tvector::normalize() {
    double len = length();

    // null vectors cannot be normalized
    if (len == 0)
        return;

    pos[0] /= len;
    pos[1] /= len;
    pos[2] /= len;
}


/// Normalizes the vector to a length of 1.
/// \date    2013-08-02
Ped::Tvector Ped::Tvector::normalized() const {
    double len = length();

    // null vectors cannot be normalized
    if (len == 0)
        return Ped::Tvector();;

    return Ped::Tvector(pos[0]/len, pos[1]/len, pos[2]/len);
}


/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
double Ped::Tvector::scalar(const Ped::Tvector &a, const Ped::Tvector &b) {
    return acos( Ped::Tvector::dotProduct(a, b) / ( a.length() * b.length() ) );
}


/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
double Ped::Tvector::dotProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
    return (a.pos[0]*b.pos[0] + a.pos[1]*b.pos[1] + a.pos[2]*b.pos[2]);
}


/// Calculates the cross product of two vectors.
/// \date    2010-02-12
/// \param   &a The first vector
/// \param   &b The second vector
Ped::Tvector Ped::Tvector::crossProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
    return Ped::Tvector(
        a.pos[1]*b.pos[2] - a.pos[2]*b.pos[1],
        a.pos[2]*b.pos[1] - a.pos[0]*b.pos[2],
        a.pos[0]*b.pos[1] - a.pos[1]*b.pos[0]);
}


/// Scales this vector by a given factor in each dimension.
/// \date    2013-08-02
/// \param   factor The scalar value to multiply with.
void Ped::Tvector::scale(double factor) {
    pos[0] *= factor;
    pos[1] *= factor;
    pos[2] *= factor;
}


/// Returns a copy of this vector which is multiplied in each dimension by a given factor.
/// \date    2013-07-16
/// \return  The scaled vector.
/// \param   factor The scalar value to multiply with.
Ped::Tvector Ped::Tvector::scaled(double factor) const {
    return Ped::Tvector(factor*pos[0], factor*pos[1], factor*pos[2]);
}

Ped::Tvector Ped::Tvector::leftNormalVector() const {
    return Ped::Tvector(-pos[1], pos[0]);
}

Ped::Tvector Ped::Tvector::rightNormalVector() const {
    return Ped::Tvector(pos[1], -pos[0]);
}

double Ped::Tvector::polarRadius() const {
    return length();
}

double Ped::Tvector::polarAngle() const {
  return atan2(pos[1], pos[0]);
}

double Ped::Tvector::angleTo(const Tvector &other) const {
    double angleThis = polarAngle();
    double angleOther = other.polarAngle();

    // compute angle
    double diffAngle = angleOther - angleThis;
    // → normalize angle
    if(diffAngle > M_PI)
        diffAngle -= 2*M_PI;
    else if(diffAngle <= -M_PI)
        diffAngle += 2*M_PI;

    return diffAngle;
}

Ped::Tvector Ped::Tvector::operator+(const Tvector& other) const {
    return Ped::Tvector(
        pos[0] + other.pos[0],
        pos[1] + other.pos[1],
        pos[2] + other.pos[2]);
}

Ped::Tvector Ped::Tvector::operator-(const Tvector& other) const {
    return Ped::Tvector(
        pos[0] - other.pos[0],
        pos[1] - other.pos[1],
        pos[2] - other.pos[2]);
}

Ped::Tvector Ped::Tvector::operator*(double factor) const {
    return scaled(factor);
}

Ped::Tvector Ped::Tvector::operator/(double divisor) const {
    return scaled(1/divisor);
}

Ped::Tvector& Ped::Tvector::operator+=(const Tvector& vectorIn) {
    pos[0] += vectorIn.pos[0];
    pos[1] += vectorIn.pos[1];
    pos[2] += vectorIn.pos[2];
    return *this;
}

Ped::Tvector& Ped::Tvector::operator-=(const Tvector& vectorIn) {
    pos[0] -= vectorIn.pos[0];
    pos[1] -= vectorIn.pos[1];
    pos[2] -= vectorIn.pos[2];
    return *this;
}

Ped::Tvector& Ped::Tvector::operator*=(double factor) {
    scale(factor);
    return *this;
}

Ped::Tvector& Ped::Tvector::operator*=(const Tvector& vectorIn) {
    pos[0] *= vectorIn.pos[0];
    pos[1] *= vectorIn.pos[1];
    pos[2] *= vectorIn.pos[2];
    return *this;
}

Ped::Tvector& Ped::Tvector::operator/=(double divisor) {
    scale(1/divisor);
    return *this;
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return (vector1In.pos[0] == vector2In.pos[0])
        && (vector1In.pos[1] == vector2In.pos[1])
        && (vector1In.pos[2] == vector2In.pos[2]);
}

bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return (vector1In.pos[0] != vector2In.pos[0])
        || (vector1In.pos[1] != vector2In.pos[1])
        || (vector1In.pos[2] != vector2In.pos[2]);
}

Ped::Tvector operator+(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return Ped::Tvector(
        vector1In.pos[0] + vector2In.pos[0],
        vector1In.pos[1] + vector2In.pos[1],
        vector1In.pos[2] + vector2In.pos[2]);
}

Ped::Tvector operator-(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return Ped::Tvector(
        vector1In.pos[0] - vector2In.pos[0],
        vector1In.pos[1] - vector2In.pos[1],
        vector1In.pos[2] - vector2In.pos[2]);
}

Ped::Tvector operator-(const Ped::Tvector& vectorIn) {
    return Ped::Tvector(
        -vectorIn.pos[0],
        -vectorIn.pos[1],
        -vectorIn.pos[2]);
}

Ped::Tvector operator*(double factor, const Ped::Tvector& vector) {
    return vector.scaled(factor);
}
