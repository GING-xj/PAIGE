#cmake_minimum_required(VERSION 3.12)
#project(cereal)
#
#set(CMAKE_CXX_STANDARD 14)
#include_directories(../)


#include_directories(.)
#include_directories(archives)
#include_directories(details)
#include_directories(external)
#include_directories(external/rapidjson)
#include_directories(external/rapidjson/error)
#include_directories(external/rapidjson/internal)
#include_directories(external/rapidjson/msinttypes)
#include_directories(external/rapidxml)
#include_directories(types)
#include_directories(types/concepts)
#
#add_executable(cereal
#        archives/adapters.hpp
#        archives/binary.hpp
#        archives/json.hpp
#        archives/portable_binary.hpp
#        archives/xml.hpp
#        details/helpers.hpp
#        details/polymorphic_impl.hpp
#        details/polymorphic_impl_fwd.hpp
#        details/static_object.hpp
#        details/traits.hpp
#        details/util.hpp
#        external/rapidjson/error/en.h
#        external/rapidjson/error/error.h
#        external/rapidjson/internal/biginteger.h
#        external/rapidjson/internal/diyfp.h
#        external/rapidjson/internal/dtoa.h
#        external/rapidjson/internal/ieee754.h
#        external/rapidjson/internal/itoa.h
#        external/rapidjson/internal/meta.h
#        external/rapidjson/internal/pow10.h
#        external/rapidjson/internal/regex.h
#        external/rapidjson/internal/stack.h
#        external/rapidjson/internal/strfunc.h
#        external/rapidjson/internal/strtod.h
#        external/rapidjson/internal/swap.h
#        external/rapidjson/msinttypes/inttypes.h
#        external/rapidjson/msinttypes/stdint.h
#        external/rapidjson/allocators.h
#        external/rapidjson/document.h
#        external/rapidjson/encodedstream.h
#        external/rapidjson/encodings.h
#        external/rapidjson/filereadstream.h
#        external/rapidjson/filewritestream.h
#        external/rapidjson/fwd.h
#        external/rapidjson/istreamwrapper.h
#        external/rapidjson/memorybuffer.h
#        external/rapidjson/memorystream.h
#        external/rapidjson/ostreamwrapper.h
#        external/rapidjson/pointer.h
#        external/rapidjson/prettywriter.h
#        external/rapidjson/rapidjson.h
#        external/rapidjson/reader.h
#        external/rapidjson/schema.h
#        external/rapidjson/stream.h
#        external/rapidjson/stringbuffer.h
#        external/rapidjson/writer.h
#        external/rapidxml/rapidxml.hpp
#        external/rapidxml/rapidxml_iterators.hpp
#        external/rapidxml/rapidxml_print.hpp
#        external/rapidxml/rapidxml_utils.hpp
#        external/base64.hpp
#        types/concepts/pair_associative_container.hpp
#        types/array.hpp
#        types/base_class.hpp
#        types/bitset.hpp
#        types/boost_variant.hpp
#        types/chrono.hpp
#        types/common.hpp
#        types/complex.hpp
#        types/deque.hpp
#        types/forward_list.hpp
#        types/functional.hpp
#        types/list.hpp
#        types/map.hpp
#        types/memory.hpp
#        types/polymorphic.hpp
#        types/queue.hpp
#        types/set.hpp
#        types/stack.hpp
#        types/string.hpp
#        types/tuple.hpp
#        types/unordered_map.hpp
#        types/unordered_set.hpp
#        types/utility.hpp
#        types/valarray.hpp
#        types/vector.hpp
#        access.hpp
#        cereal.hpp
#        macros.hpp)
