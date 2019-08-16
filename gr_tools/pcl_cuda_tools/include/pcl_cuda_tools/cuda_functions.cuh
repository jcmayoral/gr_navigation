#ifndef TOOLS_CUH
#define TOOLS_CUH

extern "C"
{
  int do_cuda_stuff(int *o_x, int size);
  void stop_cuda_stuff(int *x, int *t);
}

#endif
