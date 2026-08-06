%module CSF
%{
  #define SWIG_FILE_WITH_INIT

  #include "../src/CSF.h"
  #include "../src/Cloth.h"
%}


%include "stdint.i"
%include "std_string.i"
%include "std_vector.i"
%include "numpy.i"

%init %{
import_array();
%}

namespace std
{
    %template(VecInt) vector<int>;
    %template(VecFloat) vector<float>;
    %template(VecVecFloat) vector< vector<float> >;
    %template(VecDouble) vector<double>;
}


%apply (unsigned char* INPLACE_ARRAY1, int DIM1) {(uint8_t* ground_mask, int n_points)};
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(double *points, int rows, int cols)};

%include "../src/Cloth.h"
%include "../src/CSF.h"

