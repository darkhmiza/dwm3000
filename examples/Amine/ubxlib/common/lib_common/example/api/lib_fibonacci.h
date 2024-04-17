#ifndef _LIB_FIBONACCI_H_
#define _LIB_FIBONACCI_H_

/**
 * Returns the n'th number of the fibonacci series
 * @param ctx       the library context given when library was opened
 * @param series    the number in the series to return
 * @return the given number in the fibonacci series or negative error.
 */
int libFibCalc(void *ctx, int series);

/**
 * Returns the result of the last calculation
 * @param ctx       the library context given when library was opened
 * @return -1 if series is negative, else the given number in the fibonacci series.
 */
int libFibLastRes(void *ctx);

/**
 * Returns a hello world string
 * @param ctx       the library context given when library was opened
 * @return Hello world.
 */
const char *libFibHelloWorld(void *ctx);

#endif // _LIB_FIBONACCI_H_