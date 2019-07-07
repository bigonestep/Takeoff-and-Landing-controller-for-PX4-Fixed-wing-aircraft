#ifndef _MIXER_TVC_TABLES
#define _MIXER_TVC_TABLES

enum class TvcGeometry : TvcGeometryUnderlyingType {
	QUAD_X,                 // 4 fins in x configuration
	MAX_GEOMETRY
}; // enum class TvcGeometry

namespace
{
const TvcMixer::Alloc_Vect _config_quad_x[] = {
	{ -1.000000,  1.000000,  1.000000 },
	{ -1.000000, -1.000000,  1.000000 },
	{  1.000000, -1.000000,  1.000000 },
	{  1.000000,  1.000000,  1.000000 },
};

const TvcMixer::Alloc_Vect *_config_index[] = {
	&_config_quad_x[0],
};

const unsigned _config_actuator_count[] = {
	5, /* quad_x */
};

const char *_config_key[] = {
	"4x",	/* quad_x */
};

} // anonymous namespace

#endif /* _MIXER_TVC_TABLES */
