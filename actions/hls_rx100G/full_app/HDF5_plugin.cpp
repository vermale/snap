#include <hdf5.h>
#include "JFReceiver.h"

size_t jf_filter(unsigned int flags, size_t cd_nelmts,
	const unsigned int cd_values[], size_t nbytes,
	size_t *buf_size, void **buf)
{
	return 1;
}

H5Z_class_t H5Z_JF[1] = {{
        H5Z_CLASS_T_VERS,       /* H5Z_class_t version */
        (H5Z_filter_t)H5Z_FILTER_JF,         /* Filter id number             */
        1,              /* encoder_present flag (set to true) */
        1,              /* decoder_present flag (set to true) */
        "HDF5 JUNGFRAU detector filter",
        /* Filter name for debugging    */
        NULL,                       /* The "can apply" callback     */
        NULL,                       /* The "set local" callback     */
        (H5Z_func_t)jf_filter,         /* The actual filter function   */
}};

//H5PL_type_t H5PLget_plugin_type(void) {
//	return H5PL_TYPE_FILTER;
//}

const void *H5PLget_plugin_info(void) {
	return H5Z_JF;
}


