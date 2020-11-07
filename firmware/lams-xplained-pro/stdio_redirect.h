#ifndef STDIO_REDIRECT_H_
#define STDIO_REDIRECT_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdio.h>
#include <stdio_io.h>

void STDIO_REDIRECT_example(void);

/**
 * \brief Initialize STDIO Redirect
 */
void stdio_redirect_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STDIO_REDIRECT_H_ */
