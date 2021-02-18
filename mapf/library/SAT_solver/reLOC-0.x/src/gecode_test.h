/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.21-robik                              */
/*                                                                            */
/*                      (C) Copyright 2019 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* gecode_test.h / 0.21-robik_020                                             */
/*----------------------------------------------------------------------------*/
//
// Gecode solver intergration - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __GECODE_TEST_H__
#define __GECODE_TEST_H__

#include <gecode/int.hh>
#include <gecode/search.hh>

#include "reloc.h"


using namespace sReloc;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    class sGecodeTestSpace
    : public Space
    {
    public:
	sGecodeTestSpace(int b, int c);

    public:
	IntVar m_a, m_b, m_c;
    };


/*----------------------------------------------------------------------------*/

    void test_gecode_solver_1(void);

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __GECODE_TEST_H__ */
