#ifndef COMPILER_H_
#define COMPILER_H_

/* The memory class is used for the declaration of local pointers */
#define AUTOMATIC

/* The memory class is used within type definitions, where no memory qualifier can be specified */
#define TYPEDEF

/* This is used to define the void pointer to zero definition */
#define NULL_PTR          ((void *)0)

/* This is used to define the abstraction of compiler keyword inline */
#define INLINE            inline

/* This is used to define the local inline function */
#define LOCAL_INLINE      static inline

/* This is used to define the abstraction of compiler keyword static */
#define STATIC            static

#endif /* COMPILER_H_ */
