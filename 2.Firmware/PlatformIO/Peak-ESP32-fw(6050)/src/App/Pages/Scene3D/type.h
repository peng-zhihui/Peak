#ifndef TYPE_H
#define TYPE_H

// ----------------------------------------------
// defines
// ----------------------------------------------
#define PRES             16384
#define PSHIFT           14
#define PROUNDBIT        (1 << (PSHIFT-1))

// ----------------------------------------------
// typedef
// ----------------------------------------------
 struct Vector3
{
  long x, y, z;
} ;

 struct Vector3i
{
  int x, y, z;
} ;

// fixed point identity matrix
 struct Matrix4
{
  long m[4][4] = {
      {PRES,    0,    0,    0},
      {   0, PRES,    0,    0},
      {   0,    0, PRES,    0},
      {   0,    0,    0, PRES}
  };
} ;

// ----------------------------------------------
// functions
// ----------------------------------------------
// fixed point multiplication
static long pMultiply(long x, long y) {
  return ( (x * y) + PROUNDBIT) >> PSHIFT;
}

#endif // TYPE_H
