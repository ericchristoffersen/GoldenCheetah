/*
 * Module code.
 *
 * Generated by SIP 4.19.7
 */

#include "sipAPIgoldencheetah.h"

#line 59 "goldencheetah.sip"
#include "Bindings.h"
#line 12 "./sipgoldencheetahcmodule.cpp"
#line 244 "goldencheetah.sip"
#include "Bindings.h"
#line 15 "./sipgoldencheetahcmodule.cpp"
#line 334 "goldencheetah.sip"
//#include "Bindings.h"
#line 18 "./sipgoldencheetahcmodule.cpp"

/* Define the strings used by this module. */
const char sipStrings_goldencheetah[] = {
    'd', 'e', 'l', 'e', 't', 'e', 'A', 'c', 't', 'i', 'v', 'i', 't', 'y', 'S', 'a', 'm', 'p', 'l', 'e', 0,
    'c', 'r', 'e', 'a', 't', 'e', 'X', 'D', 'a', 't', 'a', 'S', 'e', 'r', 'i', 'e', 's', 0,
    'a', 'c', 't', 'i', 'v', 'i', 't', 'y', 'I', 'n', 't', 'e', 'r', 'v', 'a', 'l', 's', 0,
    'P', 'y', 't', 'h', 'o', 'n', 'X', 'D', 'a', 't', 'a', 'S', 'e', 'r', 'i', 'e', 's', 0,
    'P', 'y', 't', 'h', 'o', 'n', 'D', 'a', 't', 'a', 'S', 'e', 'r', 'i', 'e', 's', 0,
    's', 'e', 'a', 's', 'o', 'n', 'I', 'n', 't', 'e', 'r', 'v', 'a', 'l', 's', 0,
    'a', 'c', 't', 'i', 'v', 'i', 't', 'y', 'M', 'e', 'a', 'n', 'm', 'a', 'x', 0,
    'a', 'c', 't', 'i', 'v', 'i', 't', 'y', 'M', 'e', 't', 'r', 'i', 'c', 's', 0,
    's', 'e', 'a', 's', 'o', 'n', 'M', 'e', 'a', 's', 'u', 'r', 'e', 's', 0,
    'a', 'd', 'd', 'A', 'n', 'n', 'o', 't', 'a', 't', 'i', 'o', 'n', 0,
    's', 'e', 'a', 's', 'o', 'n', 'M', 'e', 'a', 'n', 'm', 'a', 'x', 0,
    's', 'e', 'a', 's', 'o', 'n', 'M', 'e', 't', 'r', 'i', 'c', 's', 0,
    's', 'e', 'r', 'i', 'e', 's', 'P', 'r', 'e', 's', 'e', 'n', 't', 0,
    'g', 'o', 'l', 'd', 'e', 'n', 'c', 'h', 'e', 'e', 't', 'a', 'h', 0,
    'd', 'e', 'l', 'e', 't', 'e', 'S', 'e', 'r', 'i', 'e', 's', 0,
    'a', 'c', 't', 'i', 'v', 'i', 't', 'y', 'W', 'b', 'a', 'l', 0,
    'a', 't', 'h', 'l', 'e', 't', 'e', 'Z', 'o', 'n', 'e', 's', 0,
    'c', 'o', 'n', 'f', 'i', 'g', 'C', 'h', 'a', 'r', 't', 0,
    'o', 'r', 'i', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', 0,
    'p', 'o', 's', 't', 'P', 'r', 'o', 'c', 'e', 's', 's', 0,
    's', 'e', 'a', 's', 'o', 'n', 'P', 'e', 'a', 'k', 's', 0,
    'x', 'd', 'a', 't', 'a', 'S', 'e', 'r', 'i', 'e', 's', 0,
    'Q', 'S', 't', 'r', 'i', 'n', 'g', 'L', 'i', 's', 't', 0,
    '_', '_', 's', 'e', 't', 'i', 't', 'e', 'm', '_', '_', 0,
    '_', '_', 'g', 'e', 't', 'i', 't', 'e', 'm', '_', '_', 0,
    'c', 'o', 'n', 'f', 'i', 'g', 'A', 'x', 'i', 's', 0,
    'c', 'a', 't', 'e', 'g', 'o', 'r', 'i', 'e', 's', 0,
    'l', 'a', 'b', 'e', 'l', 'c', 'o', 'l', 'o', 'r', 0,
    's', 'y', 'm', 'b', 'o', 'l', 's', 'i', 'z', 'e', 0,
    's', 'e', 'r', 'i', 'e', 's', 'U', 'n', 'i', 't', 0,
    'x', 'd', 'a', 't', 'a', 'N', 'a', 'm', 'e', 's', 0,
    's', 'e', 'r', 'i', 'e', 's', 'L', 'a', 's', 't', 0,
    's', 'e', 'r', 'i', 'e', 's', 'N', 'a', 'm', 'e', 0,
    'a', 'c', 't', 'i', 'v', 'i', 't', 'i', 'e', 's', 0,
    'p', 'r', 'o', 'c', 'e', 's', 's', 'o', 'r', 0,
    's', 'e', 'a', 's', 'o', 'n', 'P', 'm', 'c', 0,
    's', 'e', 't', 'C', 'u', 'r', 'v', 'e', 0,
    'd', 'u', 'r', 'a', 't', 'i', 'o', 'n', 0,
    'a', 'c', 't', 'i', 'v', 'i', 't', 'y', 0,
    't', 'h', 'r', 'e', 'a', 'd', 'i', 'd', 0,
    'B', 'i', 'n', 'd', 'i', 'n', 'g', 's', 0,
    'v', 'i', 's', 'i', 'b', 'l', 'e', 0,
    'o', 'p', 'a', 'c', 'i', 't', 'y', 0,
    'y', 's', 'e', 'r', 'i', 'e', 's', 0,
    'x', 's', 'e', 'r', 'i', 'e', 's', 0,
    'a', 'n', 'i', 'm', 'a', 't', 'e', 0,
    'm', 'e', 't', 'r', 'i', 'c', 's', 0,
    'c', 'o', 'm', 'p', 'a', 'r', 'e', 0,
    'a', 't', 'h', 'l', 'e', 't', 'e', 0,
    'w', 'e', 'b', 'p', 'a', 'g', 'e', 0,
    'v', 'e', 'r', 's', 'i', 'o', 'n', 0,
    '_', '_', 'l', 'e', 'n', '_', '_', 0,
    '_', '_', 's', 't', 'r', '_', '_', 0,
    'Q', 'S', 't', 'r', 'i', 'n', 'g', 0,
    'l', 'e', 'g', 'e', 'n', 'd', 0,
    'o', 'p', 'e', 'n', 'g', 'l', 0,
    's', 'y', 'm', 'b', 'o', 'l', 0,
    'c', 'o', 'l', 'o', 'r', 's', 0,
    'l', 'a', 'b', 'e', 'l', 's', 0,
    'l', 'e', 'g', 'p', 'o', 's', 0,
    'm', 'e', 't', 'r', 'i', 'c', 0,
    's', 'e', 'a', 's', 'o', 'n', 0,
    'f', 'i', 'l', 't', 'e', 'r', 0,
    'r', 'e', 's', 'u', 'l', 't', 0,
    'r', 'e', 'm', 'o', 'v', 'e', 0,
    'a', 'p', 'p', 'e', 'n', 'd', 0,
    'a', 'l', 'i', 'g', 'n', 0,
    'y', 'n', 'a', 'm', 'e', 0,
    'x', 'n', 'a', 'm', 'e', 0,
    's', 't', 'a', 'c', 'k', 0,
    't', 'i', 't', 'l', 'e', 0,
    'i', 'n', 'd', 'e', 'x', 0,
    'g', 'r', 'o', 'u', 'p', 0,
    'x', 'd', 'a', 't', 'a', 0,
    's', 'p', 'o', 'r', 't', 0,
    'v', 'a', 'l', 'u', 'e', 0,
    'b', 'u', 'i', 'l', 'd', 0,
    'l', 'i', 'n', 'e', 0,
    'j', 'o', 'i', 'n', 0,
    't', 'y', 'p', 'e', 0,
    'd', 'a', 't', 'e', 0,
    'l', 'o', 'g', 0,
    'm', 'i', 'n', 0,
    'a', 'l', 'l', 0,
    'u', 'r', 'l', 0,
    's', '2', 0,
    's', '1', 0,
};


/*
 * This defines each type in this module.
 */
sipTypeDef *sipExportedTypes_goldencheetah[] = {
    &sipTypeDef_goldencheetah_Bindings.ctd_base,
    &sipTypeDef_goldencheetah_PythonDataSeries.ctd_base,
    &sipTypeDef_goldencheetah_PythonXDataSeries.ctd_base,
    &sipTypeDef_goldencheetah_QString.mtd_base,
    &sipTypeDef_goldencheetah_QStringList.mtd_base,
};


/* This defines this module. */
sipExportedModuleDef sipModuleAPI_goldencheetah = {
    0,
    SIP_API_MINOR_NR,
    sipNameNr_goldencheetah,
    0,
    sipStrings_goldencheetah,
    NULL,
    NULL,
    5,
    sipExportedTypes_goldencheetah,
    NULL,
    0,
    NULL,
    0,
    NULL,
    NULL,
    NULL,
    {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};


/* The SIP API and the APIs of any imported modules. */
const sipAPIDef *sipAPI_goldencheetah;


/* The Python module initialisation function. */
#if PY_MAJOR_VERSION >= 3
#define SIP_MODULE_ENTRY        PyInit_goldencheetah
#define SIP_MODULE_TYPE         PyObject *
#define SIP_MODULE_DISCARD(r)   Py_DECREF(r)
#define SIP_MODULE_RETURN(r)    return (r)
#else
#define SIP_MODULE_ENTRY        initgoldencheetah
#define SIP_MODULE_TYPE         void
#define SIP_MODULE_DISCARD(r)
#define SIP_MODULE_RETURN(r)    return
#endif

#if defined(SIP_STATIC_MODULE)
extern "C" SIP_MODULE_TYPE SIP_MODULE_ENTRY()
#else
PyMODINIT_FUNC SIP_MODULE_ENTRY()
#endif
{
    static PyMethodDef sip_methods[] = {
        {0, 0, 0, 0}
    };

#if PY_MAJOR_VERSION >= 3
    static PyModuleDef sip_module_def = {
        PyModuleDef_HEAD_INIT,
        "goldencheetah",
        NULL,
        -1,
        sip_methods,
        NULL,
        NULL,
        NULL,
        NULL
    };
#endif

    PyObject *sipModule, *sipModuleDict;
    PyObject *sip_sipmod, *sip_capiobj;

    /* Initialise the module and get it's dictionary. */
#if PY_MAJOR_VERSION >= 3
    sipModule = PyModule_Create(&sip_module_def);
#elif PY_VERSION_HEX >= 0x02050000
    sipModule = Py_InitModule(sipName_goldencheetah, sip_methods);
#else
    sipModule = Py_InitModule(const_cast<char *>(sipName_goldencheetah), sip_methods);
#endif

    if (sipModule == NULL)
        SIP_MODULE_RETURN(NULL);

    sipModuleDict = PyModule_GetDict(sipModule);

    /* Get the SIP module's API. */
#if PY_VERSION_HEX >= 0x02050000
    sip_sipmod = PyImport_ImportModule(SIP_MODULE_NAME);
#else
    sip_sipmod = PyImport_ImportModule(const_cast<char *>(SIP_MODULE_NAME));
#endif

    if (sip_sipmod == NULL)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(NULL);
    }

    sip_capiobj = PyDict_GetItemString(PyModule_GetDict(sip_sipmod), "_C_API");
    Py_DECREF(sip_sipmod);

#if defined(SIP_USE_PYCAPSULE)
    if (sip_capiobj == NULL || !PyCapsule_CheckExact(sip_capiobj))
#else
    if (sip_capiobj == NULL || !PyCObject_Check(sip_capiobj))
#endif
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(NULL);
    }

#if defined(SIP_USE_PYCAPSULE)
    sipAPI_goldencheetah = reinterpret_cast<const sipAPIDef *>(PyCapsule_GetPointer(sip_capiobj, SIP_MODULE_NAME "._C_API"));
#else
    sipAPI_goldencheetah = reinterpret_cast<const sipAPIDef *>(PyCObject_AsVoidPtr(sip_capiobj));
#endif

#if defined(SIP_USE_PYCAPSULE)
    if (sipAPI_goldencheetah == NULL)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(NULL);
    }
#endif

    /* Export the module and publish it's API. */
    if (sipExportModule(&sipModuleAPI_goldencheetah,SIP_API_MAJOR_NR,SIP_API_MINOR_NR,0) < 0)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(0);
    }
    /* Initialise the module now all its dependencies have been set up. */
    if (sipInitModule(&sipModuleAPI_goldencheetah,sipModuleDict) < 0)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(0);
    }

    SIP_MODULE_RETURN(sipModule);
}
