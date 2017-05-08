#ifndef HEADER_H
#define HEADER_H
#include <iostream>
#include <assert.h>
#include <iterator>
#include <string>
#include <boost/static_assert.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/mpl/or.hpp>
#include <boost/type_traits.hpp>
#include <boost/type_traits/add_reference.hpp>
#include <boost/type_traits/is_stateless.hpp>

#include <boost/mpl/vector.hpp>
#include <boost/mpl/vector_c.hpp>
#include <boost/mpl/integral_c.hpp>
#include <boost/mpl/equal.hpp>
#include <boost/mpl/equal_to.hpp>
#include <boost/mpl/replace.hpp>
#include <boost/mpl/replace_if.hpp>
#include <boost/mpl/placeholders.hpp>

#include <boost/mpl/size.hpp>
#include <boost/mpl/advance.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/push_back.hpp>
#include <boost/mpl/front.hpp>
#include <boost/mpl/back.hpp>

#include <boost/mpl/count_if.hpp>

namespace mpl=boost::mpl;
using namespace std;
using namespace mpl::placeholders;
#endif

