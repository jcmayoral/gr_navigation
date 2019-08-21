#ifndef FILTER_TOOLS_CUH
#define FILTER_TOOLS_CUH

extern "C"
{
  int apply_cuda_filter(float *o_x, float *o_y, float *o_z, float min_limit, float max_limit, float filter_value, int size);
  void free_memory(float *x);
}

#endif
