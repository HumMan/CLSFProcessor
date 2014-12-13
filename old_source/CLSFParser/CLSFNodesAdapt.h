#include <boost/fusion/include/adapt_struct.hpp>
#include <string>
#include <vector>

BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_COMMENT,
						  (std::string, text)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_TOOL_PATH,
						  (std::string, path_name)
						  (std::string, tool_type)
						  (std::string, tool_name)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_TOOL_DATA,
						  (std::string, tool_type)
						  (std::vector<double>, params)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_SELECT,
						  (std::string, name)
						  (int, id)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_MSYS,
						  (std::vector<double>, params)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_GOTO,
						  (double, pos[0])
						  (double, pos[1])
						  (double, pos[2])
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_VEC3,
						  (double, v[0])
						  (double, v[1])
						  (double, v[2])
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_FROM,
						  (double, pos[0])
						  (double, pos[1])
						  (double, pos[2])
						  )


						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_GOHOME,
						  (double, pos[0])
						  (double, pos[1])
						  (double, pos[2])
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_CIRCLE,
						  (double, center[0])
						  (double, center[1])
						  (double, center[2])
						  (double, normal[0])
						  (double, normal[1])
						  (double, normal[2])
						  (std::vector<double>, params)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_FEEDRAT,
						  (double, feed_rate)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_SPINDL,
						  (std::string, type)
						  (double, rate)
						  (std::string, dir)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_CUTCOM,
						  (std::string, type)
						  (std::string, plane)
						  )

						  BOOST_FUSION_ADAPT_STRUCT(
						  TCLSF_AUXFUN,
						  (int, id)
						  (std::string, text)
						  )

						  //BOOST_FUSION_ADAPT_STRUCT(
						  //						  TCLSF_END_OF_PATH,
						  //						  )