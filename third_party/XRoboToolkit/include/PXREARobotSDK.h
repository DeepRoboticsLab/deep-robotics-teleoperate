#ifndef PXREACLIENTSDK_H
#define PXREACLIENTSDK_H
#ifdef _WIN32
#if defined(PXREACLIENTSDK_LIBRARY)
#  define PXREACLIENTSDK_EXPORT __declspec(dllexport)
#else
#  define PXREACLIENTSDK_EXPORT __declspec(dllimport)
#endif
#endif

#ifdef __linux__
#if defined(PXREACLIENTSDK_LIBRARY)
#  define PXREACLIENTSDK_EXPORT __attribute__((visibility("default")))
#else
#  define PXREACLIENTSDK_EXPORT __attribute__((visibility("default")))
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum PXREAClientCallbackType
{
    /// @brief 服务已连接
    PXREAServerConnect          = 1<<2,
    /// @brief 服务已断开
    PXREAServerDisconnect       = 1<<3,
    /// @brief 设备上线
    PXREADeviceFind             = 1<<4,
    /// @brief 设备离线
    PXREADeviceMissing          = 1<<5,
    /// @brief 设备连接
    PXREADeviceConnect          = 1<<9,
    /// @brief 设备状态Json描述
    PXREADeviceStateJson        = 1<<25,
    /// @brief 掩码,用于开启全部回调
    PXREAFullMask               = 0xffffffff
};








/// @brief 设备状态json描述
typedef struct {
    /// @brief 设备sn
    char devID[32];
    /// @brief json格式的设备状态
    char stateJson[16352];
}PXREADevStateJson;







/**
 * @brief 客户端回调，用户接收服务端消息
 * @param context 回调上下文，由 #Init 参数1 context 传入
 * @param type 回调类型
 * @param status 回调状态码
 * @param userData 回调数据指针，由参数2 type 决定
 */
typedef void(*pfPXREAClientCallback)(void* context,PXREAClientCallbackType type,int status,void* userData);

/**
 * @brief SDK初始化接口
 * @brief 连接服务，同时注册回调
 * @param context 回调上下文，用于为回调函数传入用户自定义数据
 * @param cliCallback 回调函数指针，用于监听服务端消息
 * @param mask 回调掩码，用于屏蔽某些服务端消息
 */
PXREACLIENTSDK_EXPORT int PXREAInit(void* context,pfPXREAClientCallback cliCallback,unsigned mask);
/**
 * @brief 终止接口
 * @brief 断开服务连接
 */
PXREACLIENTSDK_EXPORT int PXREADeinit();

/**
 * @brief 向设备发送json格式指令
 * @param devID 设备sn
 * @param parameterJson 功能及参数,json格式,具体用法参考企业套件SDK文档
 * @return 0 成功
 * @return -1 失败
 */
PXREACLIENTSDK_EXPORT int PXREADeviceControlJson(const char *devID,const char *parameterJson);

#ifdef __cplusplus
}
#endif

#endif // PXREACLIENTSDK_H
