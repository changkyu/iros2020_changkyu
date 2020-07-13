#include <string>
#include <iostream>

#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include "utils/utils_python.hpp"

using namespace std;

namespace utils
{

void SaveTSDF2Numpy(const char* filepath, tsdf_t &tsdf)
{
    assert(tsdf.size()>0       && "tsdf.size() > 0"       );
    assert(tsdf[0].size()>0    && "tsdf[0].size() > 0"    );
    assert(tsdf[0][0].size()>0 && "tsdf[0][0[].size() > 0");    

    int dim1 = tsdf.size();
    int dim2 = tsdf[0].size();
    int dim3 = tsdf[0][0].size();

    if(PyArray_API == NULL)
    {
        char program[256];
        strcpy(program,"SaveNumpy");

        Py_SetProgramName(program);
        Py_Initialize();
        import_array();
    } 

    PyObject *pFilepath = PyUnicode_FromString(filepath);
    if( !pFilepath )
    {
        cerr << "Invalid filepath: " << filepath << endl;
        return;
    }

    float tmp[dim1][dim2][dim3];
    for( int d1=0; d1<dim1; d1++ )
    for( int d2=0; d2<dim2; d2++ )
    for( int d3=0; d3<dim3; d3++ )
    {
        tmp[d1][d2][d3] = tsdf[d1][d2][d3];
    }

    const int ND = 3;
    npy_intp dims[ND] = {dim1, dim2, dim3};

    PyObject *pArray = PyArray_SimpleNewFromData(
        ND, dims, NPY_FLOAT32, reinterpret_cast<void*>(tmp));
    if( !pArray )
    {
        cerr << "Invalid tsdf ND=" << ND << endl;
        return;
    }

    const char *module_numpy = "numpy";
    PyObject *pName = PyUnicode_FromString(module_numpy);
    if( !pName )
    {
        cerr << "Invalid module name: " << module_numpy << endl;
        return;
    }    

    PyObject *pModule = PyImport_Import(pName);
    if( !pModule )
    {
        cerr << "Cannot import module: " << module_numpy << endl;
        return;
    }

#if 1
    const char *func_name = "save";
    PyObject *pFunc = PyObject_GetAttrString(pModule, func_name);
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        cerr << module_numpy << "." << func_name << " is not callable." << endl;
        if(pFunc) Py_DECREF(pFunc);
        return;
    }

    PyArrayObject *np_arr = reinterpret_cast<PyArrayObject*>(pArray);
    PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc, pFilepath, np_arr, NULL);
#else    
    PyObject *pMethod = PyUnicode_FromString("tofile");
    PyObject *pReturn = PyObject_CallMethodObjArgs(pArray, pMethod, pFilepath, NULL);
#endif

    return;    
}

void LoadTSDF2Numpy(const char* filepath, tsdf_t &voxel, const string &load_funct)
{
    if(PyArray_API == NULL)
    {
        char program[256];
        strcpy(program,"LoadNumpy");

        Py_SetProgramName(program);
        Py_Initialize();
        import_array();
    } 

    cout << filepath << endl;

    PyObject *pFilepath = PyUnicode_FromString(filepath);
    if( !pFilepath )
    {
        cerr << "Invalid filepath: " << filepath << endl;
        return;
    }

    const char *module_scipyio = "numpy";
    PyObject *pModuleName = PyUnicode_FromString(module_scipyio);
    if( !pModuleName )
    {
        cerr << "Invalid module name: " << module_scipyio << endl;
        return;
    }    

    PyObject *pModule = PyImport_Import(pModuleName);
    if( !pModule )
    {
        cerr << "Cannot import module: " << module_scipyio << endl;
        return;
    }

    const char *func_name = load_funct.c_str();
    PyObject *pFunc = PyObject_GetAttrString(pModule, func_name);
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        cerr << module_scipyio << "." << func_name << " is not callable." << endl;
        if(pFunc) Py_DECREF(pFunc);
        return;
    }
    
    PyArrayObject *pArray
     = (PyArrayObject*)PyObject_CallFunctionObjArgs(pFunc, pFilepath, NULL);

    PyArray_Descr* pDesc = PyArray_DTYPE(pArray);
    npy_intp* dims = PyArray_DIMS(pArray);

    voxel.resize(dims[0]);
    for( int d1=0; d1<dims[0]; d1++ )
    {
        voxel[d1].resize(dims[1]);
        for( int d2=0; d2<dims[1]; d2++ )
        {
            voxel[d1][d2].resize(dims[2]);
            for( int d3=0; d3<dims[2]; d3++ )
            {
                if( pDesc->type == 'e' )
                {
                    uint16_t* num16
                     = (uint16_t*)PyArray_GETPTR3(pArray, d1,d2,d3);
                    voxel[d1][d2][d3]
                     = utils::float16tofloat32((char*)&num16);
                }
                else if( pDesc->type == 'f' )
                {                    
                    voxel[d1][d2][d3]
                     = *(float*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else if( pDesc->type == 'd' )
                {
                    voxel[d1][d2][d3]
                     = *(double*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else
                {
                    cerr << "[Error] unknown type: " << pDesc->type << endl;
                    return;
                }
            }
        }
    }

    return;    
}

void SaveVoxel2Numpy(const char* filepath, const voxel_t &voxel)
{
    assert(voxel.size()>0       && "tsdf.size() > 0"       );
    assert(voxel[0].size()>0    && "tsdf[0].size() > 0"    );
    assert(voxel[0][0].size()>0 && "tsdf[0][0[].size() > 0");    

    int dim1 = voxel.size();
    int dim2 = voxel[0].size();
    int dim3 = voxel[0][0].size();

    if(PyArray_API == NULL)
    {
        char program[256];
        strcpy(program,"SaveNumpy");

        Py_SetProgramName(program);
        Py_Initialize();
        import_array();
    } 

    PyObject *pFilepath = PyUnicode_FromString(filepath);
    if( !pFilepath )
    {
        cerr << "Invalid filepath: " << filepath << endl;
        return;
    }

    uint8_t tmp[dim1][dim2][dim3];
    for( int d1=0; d1<dim1; d1++ )
    for( int d2=0; d2<dim2; d2++ )
    for( int d3=0; d3<dim3; d3++ )
    {
        tmp[d1][d2][d3] = voxel[d1][d2][d3];
    }

    const int ND = 3;
    npy_intp dims[ND] = {dim1, dim2, dim3};

    PyObject *pArray = PyArray_SimpleNewFromData(
        ND, dims, NPY_UINT8, reinterpret_cast<void*>(tmp));
    if( !pArray )
    {
        cerr << "Invalid voxel ND=" << ND << endl;
        return;
    }

    const char *module_numpy = "numpy";
    PyObject *pName = PyUnicode_FromString(module_numpy);
    if( !pName )
    {
        cerr << "Invalid module name: " << module_numpy << endl;
        return;
    }    

    PyObject *pModule = PyImport_Import(pName);
    if( !pModule )
    {
        cerr << "Cannot import module: " << module_numpy << endl;
        return;
    }

#if 1
    const char *func_name = "save";
    PyObject *pFunc = PyObject_GetAttrString(pModule, func_name);
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        cerr << module_numpy << "." << func_name << " is not callable." << endl;
        if(pFunc) Py_DECREF(pFunc);
        return;
    }

    PyArrayObject *np_arr = reinterpret_cast<PyArrayObject*>(pArray);
    PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc, pFilepath, np_arr, NULL);
#else    
    PyObject *pMethod = PyUnicode_FromString("tofile");
    PyObject *pReturn = PyObject_CallMethodObjArgs(pArray, pMethod, pFilepath, NULL);
#endif

    return;    
}

void LoadVoxel2Numpy(const char* filepath, voxel_t &voxel)
{
    if(PyArray_API == NULL)
    {
        char program[256];
        strcpy(program,"LoadNumpy");

        Py_SetProgramName(program);
        Py_Initialize();
        import_array();
    } 

    PyObject *pFilepath = PyUnicode_FromString(filepath);
    if( !pFilepath )
    {
        cerr << "Invalid filepath: " << filepath << endl;
        return;
    }

    const char *module_scipyio = "numpy";
    PyObject *pModuleName = PyUnicode_FromString(module_scipyio);
    if( !pModuleName )
    {
        cerr << "Invalid module name: " << module_scipyio << endl;
        return;
    }    

    PyObject *pModule = PyImport_Import(pModuleName);
    if( !pModule )
    {
        cerr << "Cannot import module: " << module_scipyio << endl;
        return;
    }

    const char *func_name = "load";
    PyObject *pFunc = PyObject_GetAttrString(pModule, func_name);
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        cerr << module_scipyio << "." << func_name << " is not callable." << endl;
        if(pFunc) Py_DECREF(pFunc);
        return;
    }
    
    PyArrayObject *pArray
     = (PyArrayObject*)PyObject_CallFunctionObjArgs(pFunc, pFilepath, NULL);

    PyArray_Descr* pDesc = PyArray_DTYPE(pArray);
    npy_intp* dims = PyArray_DIMS(pArray);

    voxel.resize(dims[0]);
    for( int d1=0; d1<dims[0]; d1++ )
    {
        voxel[d1].resize(dims[1]);
        for( int d2=0; d2<dims[1]; d2++ )
        {
            voxel[d1][d2].resize(dims[2]);
            for( int d3=0; d3<dims[2]; d3++ )
            {
                if( pDesc->type == 'B' )                
                {
                    voxel[d1][d2][d3]
                     = *(uint8_t*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else
                {
                    cerr << "[Error] unknown type: " << pDesc->type << endl;
                    return;
                }
            }
        }
    }

    return;    
}

void SaveBoolVoxel2Mat(const char* filepath, voxel_t &voxel, const char* name)
{
    assert(voxel.size()>0       && "voxel.size() > 0"       );
    assert(voxel[0].size()>0    && "voxel[0].size() > 0"    );
    assert(voxel[0][0].size()>0 && "voxel[0][0[].size() > 0");    

    int dim1 = voxel.size();
    int dim2 = voxel[0].size();
    int dim3 = voxel[0][0].size();

    if(PyArray_API == NULL)
    {
        char program[256];
        strcpy(program,"SaveNumpy");

        Py_SetProgramName(program);
        Py_Initialize();
        import_array();
    } 

    PyObject *pFilepath = PyUnicode_FromString(filepath);
    if( !pFilepath )
    {
        cerr << "Invalid filepath: " << filepath << endl;
        return;
    }

    PyObject *pName = PyUnicode_FromString(name);
    if( !pName )
    {
        cerr << "Invalid name: " << name << endl;
        return;
    }

    bool tmp[dim1][dim2][dim3];
    for( int d1=0; d1<dim1; d1++ )
    for( int d2=0; d2<dim2; d2++ )
    for( int d3=0; d3<dim3; d3++ )
    {
        tmp[d1][d2][d3] = voxel[d1][d2][d3];
    }

    const int ND = 3;
    npy_intp dims[ND] = {dim1, dim2, dim3};

    PyObject *pArray = PyArray_SimpleNewFromData(
        ND, dims, NPY_BOOL, reinterpret_cast<void*>(tmp));
    if( !pArray )
    {
        cerr << "Invalid voxel ND=" << ND << endl;
        return;
    }

    PyObject *pDict = PyDict_New();
    PyDict_SetItem(pDict, pName, pArray);

    const char *module_scipyio = "scipy.io";
    PyObject *pModuleName = PyUnicode_FromString(module_scipyio);
    if( !pModuleName )
    {
        cerr << "Invalid module name: " << module_scipyio << endl;
        return;
    }    

    PyObject *pModule = PyImport_Import(pModuleName);
    if( !pModule )
    {
        cerr << "Cannot import module: " << module_scipyio << endl;
        return;
    }

    const char *func_name = "savemat";
    PyObject *pFunc = PyObject_GetAttrString(pModule, func_name);
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        cerr << module_scipyio << "." << func_name << " is not callable." << endl;
        if(pFunc) Py_DECREF(pFunc);
        return;
    }

    PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc, pFilepath, pDict, NULL);

    return;    
}

void LoadBoolVoxelfromMat(const char* filepath, voxel_t &voxel, const char* name)
{
    if(PyArray_API == NULL)
    {
        char program[256];
        strcpy(program,"LoadNumpy");

        Py_SetProgramName(program);
        Py_Initialize();
        import_array();
    } 

    cout << filepath << endl;

    PyObject *pFilepath = PyUnicode_FromString(filepath);
    if( !pFilepath )
    {
        cerr << "Invalid filepath: " << filepath << endl;
        return;
    }

    PyObject *pName = PyUnicode_FromString(name);
    if( !pName )
    {
        cerr << "Invalid name: " << name << endl;
        return;
    }

    const char *module_scipyio = "scipy.io";
    PyObject *pModuleName = PyUnicode_FromString(module_scipyio);
    if( !pModuleName )
    {
        cerr << "Invalid module name: " << module_scipyio << endl;
        return;
    }    

    PyObject *pModule = PyImport_Import(pModuleName);
    if( !pModule )
    {
        cerr << "Cannot import module: " << module_scipyio << endl;
        return;
    }

    const char *func_name = "loadmat";
    PyObject *pFunc = PyObject_GetAttrString(pModule, func_name);
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        cerr << module_scipyio << "." << func_name << " is not callable." << endl;
        if(pFunc) Py_DECREF(pFunc);
        return;
    }

    PyObject *pDict = PyObject_CallFunctionObjArgs(pFunc, pFilepath, NULL);

    cout << pDict << endl;

    PyArrayObject* pArray = (PyArrayObject*)PyDict_GetItem(pDict, pName);

    PyArray_Descr* pDesc = PyArray_DTYPE(pArray);
    npy_intp* dims = PyArray_DIMS(pArray);

    voxel.resize(dims[0]);
    for( int d1=0; d1<dims[0]; d1++ )
    {
        voxel[d1].resize(dims[1]);
        for( int d2=0; d2<dims[1]; d2++ )
        {
            voxel[d1][d2].resize(dims[2]);
            for( int d3=0; d3<dims[2]; d3++ )
            {                
                if( pDesc->type == 'e' )
                {
                    uint16_t* num16
                     = (uint16_t*)PyArray_GETPTR3(pArray, d1,d2,d3);
                    voxel[d1][d2][d3]
                     = utils::float16tofloat32((char*)&num16);
                }
                else if( pDesc->type == 'f' )
                {                    
                    voxel[d1][d2][d3]
                     = *(float*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else if( pDesc->type == 'd' )
                {
                    voxel[d1][d2][d3]
                     = *(double*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else if( pDesc->type == 'b' )
                {
                    voxel[d1][d2][d3]
                     = *(char*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else if( pDesc->type == 'B' )
                {
                    voxel[d1][d2][d3]
                     = *(uint8_t*)PyArray_GETPTR3(pArray, d1,d2,d3);
                }
                else
                {
                    cerr << "[Error] unknown type: " << pDesc->type << endl;
                }
            }
        }
    }

    return;    
}


}