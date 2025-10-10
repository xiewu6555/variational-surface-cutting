#pragma once

#include <complex>
#include <cmath>
#include <string>
#include <random>
#include <limits>
#include <typeinfo>
#include <exception>

#include <vector3.h>
#include <vector2.h>

#define PI 3.1415926535897932384
typedef int Bool;

// Lightweight class for holding 3-tuples of unsigned ints
union uint3 {
    struct {
        unsigned int first;
        unsigned int second;
        unsigned int third;
    };
    unsigned int v[3];
};



// Complex numbers are useful
using Complex = std::complex<double>;
const Complex IM_I(0.0,1.0);
inline double dot(Complex x, Complex y) {
    return x.real() * y.real() + x.imag() * y.imag();
}

inline Complex inv(Complex c) {
    return std::conj(c) / std::norm(c);
}

namespace std {
    // NOTE: Technically, the lines below are illegal, because specializing standard library functions is ONLY allowed for
    // user-defined types. That being said, I don't think this will crash any planes.
    inline bool isfinite(const std::complex<double> c) { 
        return isfinite(c.real()) && isfinite(c.imag());
    }
}

// Various functions
template<typename T> T gcClamp(T val, T low, T high);
Vector3 gcClamp(Vector3 val, Vector3 low, Vector3 high);
template<typename T> bool approxEqualsAbsolute(T a, T b, double eps=1e-6);
double regularizeAngle(double theta); // Map theta in to [0,2pi)


template<typename T>
T sqr( T x ) { return x*x; }


// === Inline implementations
template<typename T> inline T gcClamp(T val, T low, T high) {
    if(val > high) return high;
    if(val < low) return low;
    return val;
}
inline Vector3 gcClamp(Vector3 val, Vector3 low, Vector3 high) {
    double x = gcClamp(val.x, low.x, high.x);
    double y = gcClamp(val.y, low.y, high.y);
    double z = gcClamp(val.z, low.z, high.z);
    return Vector3{x,y,z};
}

template<typename T> inline bool approxEqualsAbsolute(T a, T b, double eps) {

    double absA = std::abs(a);
    double absB = std::abs(b);
    double absDiff = std::abs(a - b);

    if(a == b) {
        return true;
    } else {
        return absDiff < eps;
    }

}

inline double regularizeAngle(double theta) {
    return theta - 2*PI*std::floor(theta / (2*PI));
}

template<typename T>
std::string typeNameString( T& x ) { return std::string(typeid(x).name()); }

template<typename T>
std::string typeNameString( T* x ) { return std::string(typeid(x).name()); }

template<typename T>
void safeDelete( T*& x )
{
   if( x != nullptr )
   {
      delete x;
      x = nullptr;
   }
}

template<typename T>
void safeDeleteArray( T*& x )
{
   if( x != nullptr )
   {
      delete[] x;
      x = nullptr;
   }
}

// Random number generation -----------------------------------------
extern std::random_device util_random_device;
extern std::mt19937 util_mersenne_twister;

inline double unitRand()
{
   std::uniform_real_distribution<double> dist( 0., 1. );
   return dist( util_mersenne_twister );
}

inline double randomReal( double minVal, double maxVal )
{
   std::uniform_real_distribution<double> dist( minVal, maxVal );
   return dist( util_mersenne_twister );
}

// Generate a random int in the INCLUSIVE range [lower,upper]
inline double randomInt( int lower, int upper )
{
   std::uniform_int_distribution<int> dist(lower, upper);
   return dist( util_mersenne_twister );
}

// === Printing things to strings ===
template<typename T>
inline std::string to_string(std::vector<T> const & v) {
    std::stringstream ss;
    ss << "[";
    for(size_t i = 0; i < v.size(); i++) {
        if(i > 0) {
            ss << ",";
        }
        ss << v[i];
    }
    ss << "]";

    return ss.str();
}


// === Custom error types
namespace GC {
    class FunctionalityException : public std::runtime_error
    {
        public:
            FunctionalityException(std::string msg) 
              : std::runtime_error("Missing functionaliy: " + msg) {};
    };
}