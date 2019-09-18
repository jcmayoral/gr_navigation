#ifndef FILTER_TOOLS_CUH
#define FILTER_TOOLS_CUH

extern "C"
{
  int apply_cuda_filter(float *o_z, bool *o_b, float min_limit, float max_limit, float filter_value, int size);
}

#endif
