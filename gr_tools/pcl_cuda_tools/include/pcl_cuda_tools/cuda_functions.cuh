#ifndef TOOLS_CUH
#define TOOLS_CUH

extern "C"
{
    void do_cuda_stuff(int n, unsigned char *x, int *hist, float delta);
    void stop_cuda_stuff(unsigned char *x, int *hist, float delta, int n);
}

#endif
