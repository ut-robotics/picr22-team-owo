#include "Python.h"
#include "numpy/arrayobject.h"

unsigned char lookup_table[0x1000000];

static PyObject* set_table (PyObject * Py_UNUSED(self), PyObject *args) {
	PyObject* arg1;

	PyArrayObject *lookup = NULL;

	if (!PyArg_ParseTuple(args, "O!", &PyArray_Type, &arg1))
		return NULL;

	lookup = (PyArrayObject*)PyArray_FROM_OTF(arg1, NPY_UINT8, NPY_IN_ARRAY);
	if (lookup == NULL) 
		goto fail;

	long i;
	for (i=0; i<lookup->dimensions[0]; ++i) {
		unsigned char *v	= (unsigned char*)PyArray_GETPTR1(lookup, i);
		lookup_table[i]	= *v;
	}

	Py_DECREF(lookup);
	Py_INCREF(Py_None);
	return Py_None;

	fail:
		Py_XDECREF(lookup);
		return NULL;
	return NULL;
}

static PyObject* segment (PyObject *dummy, PyObject *args) {
	PyObject *arg1=NULL, *out1=NULL, *out2=NULL, *out3=NULL, *out4=NULL;
	PyArrayObject *yuv=NULL, *segmented=NULL, *is_ball=NULL, *is_gatey=NULL, *is_gateb=NULL;

	if (!PyArg_ParseTuple(args, "OO!O!O!O!", &arg1, &PyArray_Type, &out1, &PyArray_Type, &out2, &PyArray_Type, &out3, &PyArray_Type, &out4))
		return NULL;

	yuv = (PyArrayObject*)PyArray_FROM_OTF(arg1, NPY_UINT8, NPY_IN_ARRAY);
	if (yuv == NULL) return NULL;
	segmented = (PyArrayObject*)PyArray_FROM_OTF(out1, NPY_UINT8, NPY_INOUT_ARRAY);
 	if (segmented == NULL) goto fail;
 	is_ball = (PyArrayObject*)PyArray_FROM_OTF(out2, NPY_UINT8, NPY_INOUT_ARRAY);
 	if (is_ball == NULL) goto fail;
 	is_gatey = (PyArrayObject*)PyArray_FROM_OTF(out3, NPY_UINT8, NPY_INOUT_ARRAY);
 	if (is_gatey == NULL) goto fail;
 	is_gateb = (PyArrayObject*)PyArray_FROM_OTF(out4, NPY_UINT8, NPY_INOUT_ARRAY);
 	if (is_gateb == NULL) goto fail;

	int i;
	int j;
	for (i=0; i<yuv->dimensions[0]; ++i) {
		for (j=0; j<yuv->dimensions[1]; ++j) {
			unsigned char *y = (unsigned char*)PyArray_GETPTR3(yuv, i, j, 0);
			unsigned char *u = (unsigned char*)PyArray_GETPTR3(yuv, i, j, 1);
			unsigned char *v = (unsigned char*)PyArray_GETPTR3(yuv, i, j, 2);
			
			unsigned char r = lookup_table[*y + (*u << 8) + (*v << 16)];
			
			unsigned char *o = (unsigned char*)PyArray_GETPTR2(segmented, i, j);
			*o = r;
			
			unsigned char *i_b = (unsigned char*)PyArray_GETPTR2(is_ball, i, j);
			unsigned char *i_gy = (unsigned char*)PyArray_GETPTR2(is_gatey, i, j);
			unsigned char *i_gb = (unsigned char*)PyArray_GETPTR2(is_gateb, i, j);
			
			*i_b	= r ;
			
			switch (r) {
				case 1:
					*i_b	= 255;
					*i_gy	= 0;
					*i_gb	= 0;
					break;
				case 2:
					*i_b	= 0;
					*i_gy	= 255;
					*i_gb	= 0;
					break;
				case 3:
					*i_b	= 0;
					*i_gy	= 0;
					*i_gb	= 255;
					break;
				default:
					*i_b	= 0;
					*i_gy	= 0;
					*i_gb	= 0;
			}
		}
	}

	Py_DECREF(yuv);
	Py_DECREF(segmented);
	Py_DECREF(is_ball);
	Py_DECREF(is_gatey);
	Py_DECREF(is_gateb);
	Py_INCREF(Py_None);
	return Py_None;

	fail:
		Py_XDECREF(yuv);
		PyArray_DiscardWritebackIfCopy(segmented);
		Py_XDECREF(segmented);
		PyArray_DiscardWritebackIfCopy(is_ball);
		Py_XDECREF(is_ball);
		PyArray_DiscardWritebackIfCopy(is_gatey);
		Py_XDECREF(is_gatey);
		PyArray_DiscardWritebackIfCopy(is_gateb);
		Py_XDECREF(is_gateb);
		return NULL;
}

static struct PyMethodDef methods[] = {
	{"set_table", set_table, METH_VARARGS, "(in_lookup_table)"},
	{"segment", segment, METH_VARARGS, "(in_yuv, in_lookup_table, out_segmented)"},
	{NULL, NULL, 0, NULL}
};

static struct PyModuleDef segmentDef =
{
    PyModuleDef_HEAD_INIT,
    "segment", /* name of module */
    "",          /* module documentation, may be NULL */
    -1,          /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    methods
};

PyMODINIT_FUNC PyInit_segment (void) {
	import_array();
	return PyModule_Create(&segmentDef);
}
