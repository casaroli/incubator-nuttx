/* src/platform.hpp.  Generated from platform.hpp.in by configure.  */
/* src/platform.hpp.in.  Generated from configure.ac by autoheader.  */

#include <nuttx/config.h>

/* Define to 1 if you have the `accept4' function. */
#define HAVE_ACCEPT4 1

/* Define to 1 if you have the `clock_gettime' function. */
#define HAVE_CLOCK_GETTIME 1

/* Define to 1 if you have the `fork' function. */
/* #undef HAVE_FORK */

/* Define to 1 if you have the `gethrtime' function. */
#define HAVE_GETHRTIME 1

/* Define to 1 if you have the <gssapi/gssapi_generic.h> header file. */
/* #undef HAVE_GSSAPI_GSSAPI_GENERIC_H */

/* if_nametoindex is available */
// #define HAVE_IF_NAMETOINDEX 1

/* Enabled GSSAPI security */
/* #undef HAVE_LIBGSSAPI_KRB5 */

/* The libunwind library is to be used */
/* #undef HAVE_LIBUNWIND */

/* Define to 1 if you have the `mkdtemp' function. */
#define HAVE_MKDTEMP 1

/* Define to 1 if `posix_memalign' works. */
#define HAVE_POSIX_MEMALIGN 1

/* strnlen is available */
#define HAVE_STRNLEN 1

/* Enable militant API assertions */
/* #undef ZMQ_ACT_MILITANT */

/* Provide draft classes and methods */
/* #undef ZMQ_BUILD_DRAFT_API */

/* Using "$zmq_cacheline_size" bytes alignment for lock-free data structures
   */
#define ZMQ_CACHELINE_SIZE 64

/* Force to use mutexes */
/* #undef ZMQ_FORCE_MUTEXES */

/* Whether compiler has __atomic_Xxx intrinsics. */
#define ZMQ_HAVE_ATOMIC_INTRINSICS 1

/* Using curve encryption */
#define ZMQ_HAVE_CURVE 1

#ifdef CONFIG_EVENT_FD

/* Have eventfd extension */
#define ZMQ_HAVE_EVENTFD 1

/* Whether EFD_CLOEXEC is defined and functioning. */
#define ZMQ_HAVE_EVENTFD_CLOEXEC 1

#endif

/* Whether getrandom is supported. */
#define ZMQ_HAVE_GETRANDOM 1

/* Have ifaddrs.h header. */
#define ZMQ_HAVE_IFADDRS 1

#ifdef CONFIG_ZEROMQ_IPC
/* Have AF_UNIX sockets for ipc transport */
#define ZMQ_HAVE_IPC 1
#endif

/* The libbsd library is to be used */
/* #undef ZMQ_HAVE_LIBBSD */

/* Have Linux OS */
#define ZMQ_HAVE_LINUX 1

/* Have LOCAL_PEERCRED socket option */
/* #undef ZMQ_HAVE_LOCAL_PEERCRED */

/* Have NORM protocol extension */
/* #undef ZMQ_HAVE_NORM */

/* Have OpenPGM extension */
/* #undef ZMQ_HAVE_OPENPGM */

/* Whether O_CLOEXEC is defined and functioning. */
#define ZMQ_HAVE_O_CLOEXEC 1

/* Whether pthread_setname_np() has 2 arguments */
#define ZMQ_HAVE_PTHREAD_SETNAME_2 1

/* Whether pthread_setaffinity_np() exists */
#define ZMQ_HAVE_PTHREAD_SET_AFFINITY 1

/* Whether pthread_set_name_np() exists */
#define ZMQ_HAVE_PTHREAD_SET_NAME 1

/* Whether SOCK_CLOEXEC is defined and functioning. */
#define ZMQ_HAVE_SOCK_CLOEXEC 1

/* Whether SO_BINDTODEVICE is supported. */
#define ZMQ_HAVE_SO_BINDTODEVICE 1

/* Whether SO_KEEPALIVE is supported. */
#define ZMQ_HAVE_SO_KEEPALIVE 1

/* Have SO_PEERCRED socket option */
#define ZMQ_HAVE_SO_PEERCRED 1

/* Whether SO_PRIORITY is supported. */
/* #define ZMQ_HAVE_SO_PRIORITY 1 */

/* strlcpy is available */
#define ZMQ_HAVE_STRLCPY 1

/* Whether TCP_KEEPALIVE is supported. */
#define ZMQ_HAVE_TCP_KEEPALIVE 1

/* Whether TCP_KEEPCNT is supported. */
#define ZMQ_HAVE_TCP_KEEPCNT 1

/* Whether TCP_KEEPIDLE is supported. */
#define ZMQ_HAVE_TCP_KEEPIDLE 1

/* Whether TCP_KEEPINTVL is supported. */
#define ZMQ_HAVE_TCP_KEEPINTVL 1

/* Have TIPC support */
#define ZMQ_HAVE_TIPC 1

/* Have uio.h header. */
#define ZMQ_HAVE_UIO 1

/* Using websocket */
#define ZMQ_HAVE_WS 1

/* WSS enabled */
/* #undef ZMQ_HAVE_WSS */

#if defined(CONFIG_ZEROMQ_IO_EPOLL)

/* Use 'epoll' I/O thread polling system */
#define ZMQ_IOTHREAD_POLLER_USE_EPOLL 1

#if defined(CONFIG_ZEROMQ_IO_EPOLL_CLOEXEC)

/* Use 'epoll' I/O thread polling system with CLOEXEC */
#define ZMQ_IOTHREAD_POLLER_USE_EPOLL_CLOEXEC 1

#endif /* ZEROMQ_IO_EPOLL_CLOEXEC */

#elif defined(CONFIG_ZEROMQ_IO_POLL)

/* Use 'poll' I/O thread polling system */
#define ZMQ_IOTHREAD_POLLER_USE_POLL 1

#elif defined(CONFIG_ZEROMQ_IO_SELECT)

/* Use 'select' I/O thread polling system */
#define ZMQ_IOTHREAD_POLLER_USE_SELECT 1

#endif /* CONFIG_ZEROMQ_IO_EPOLL */

/* Use 'poll' zmq_poll(er)_* API polling system */
#define ZMQ_POLL_BASED_ON_POLL 1

#if defined(CONFIG_ZEROMQ_BUILTIN_SHA1)

/* Using built-in sha1 */
#define ZMQ_USE_BUILTIN_SHA1 1

#endif

#if defined(CONFIG_ZEROMQ_CV_NONE)

/* Use no condition variable implementation. */
#define ZMQ_USE_CV_IMPL_NONE 1

#elif defined(CONFIG_ZEROMQ_CV_PTHREADS)

/* Use pthread condition variable implementation. */
#define ZMQ_USE_CV_IMPL_PTHREADS 1

#elif defined(CONFIG_ZEROMQ_CV_STL11)

/* Use stl11 condition variable implementation. */
#define ZMQ_USE_CV_IMPL_STL11 1

#endif
/* fuzz tests will be built with fuzzing engine */
// #define ZMQ_USE_FUZZING_ENGINE 1

/* Use GNUTLS for TLS */
/* #undef ZMQ_USE_GNUTLS */

/* Using libsodium for curve encryption */
/* #undef ZMQ_USE_LIBSODIUM */

/* Using NSS */
/* #undef ZMQ_USE_NSS */

/* Use radix tree implementation to manage subscriptions */
/* #undef ZMQ_USE_RADIX_TREE */

/* Using tweetnacl for curve encryption */
#define ZMQ_USE_TWEETNACL 1
