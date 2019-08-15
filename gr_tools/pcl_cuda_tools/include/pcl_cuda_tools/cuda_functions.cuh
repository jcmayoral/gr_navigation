#ifndef TOOLS_CUH
#define TOOLS_CUH

extern "C"
{
  double do_cuda_stuff(int o_x[], int n);
  void stop_cuda_stuff(int *x, int *hist);
}

#endif
