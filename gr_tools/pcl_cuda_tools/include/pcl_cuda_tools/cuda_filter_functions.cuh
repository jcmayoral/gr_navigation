#ifndef FILTER_TOOLS_CUH
#define FILTER_TOOLS_CUH

extern "C"
{
  int apply_cuda_filter(float *o_x, float *o_y, float *o_z, float limit);
  void free_memory(float *x);
}

#endif
