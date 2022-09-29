#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <errno.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "linuxrec.h"
#include "speech_recognizer.h"

#include "msp_cmn.h"
#include "qivw.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "ros/package.h"

using namespace std;

#define IVW_AUDIO_FILE_NAME "/home/castlex/castlex_ws/src/castlex_awake/bin/audio/awake.wav"
#define FRAME_LEN	640 //16k采样率的16bit音频，一帧的大小为640B, 时长20ms

#define	BUFFER_SIZE	4096
#define SAMPLE_RATE_16K     (16000)
#define SAMPLE_RATE_8K      (8000)
#define MAX_GRAMMARID_LEN   (32)
#define MAX_PARAMS_LEN      (1024)

int ISR_STATUS = 0;//oneshot专用，用来标识命令词识别结果是否已返回。

#define E_SR_NOACTIVEDEVICE		1
#define E_SR_NOMEM				2
#define E_SR_INVAL				3
#define E_SR_RECORDFAIL			4
#define E_SR_ALREADY			5

#define DEFAULT_FORMAT		\
{\
	WAVE_FORMAT_PCM,	\
	1,			\
	16000,			\
	32000,			\
	2,			\
	16,			\
	sizeof(WAVEFORMATEX)	\
}

typedef struct _UserData {
	int     build_fini;  //标识语法构建是否完成
	int     update_fini; //标识更新词典是否完成
	int     errcode; //记录语法构建或更新词典回调错误码
	char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;

static int record_state = MSP_AUDIO_SAMPLE_CONTINUE;
struct recorder *recorder;
static bool g_is_awaken_succeed = false;
static bool normal_state_flag = true;

string my_lgi_param;
string my_ssb_param;

string temp_path;
string pkg_path = ros::package::getPath("castlex_voice_system");
string fo_("fo|");
string RES_path("/bin/msc/res/asr/common.jet");
string path1 = fo_+pkg_path+RES_path;
const char * ASR_RES_PATH        = path1.data(); //离线语法识别资源路径
string BUILD_path("/bin/msc/res/asr/GrmBuilld");
string path2 = pkg_path+BUILD_path;
const char * GRM_BUILD_PATH      = path2.data(); //构建离线语法识别网络生成数据保存路径
string FILE_path("/bin/call.bnf");
string path3 = pkg_path+FILE_path;
const char * GRM_FILE = path3.data(); //构建离线识别语法网络所用的语法文件
const char * LEX_NAME            = "contact"; //更新离线识别语法的contact槽（语法文件为此示例中使用的call.bnf）

void sleep_ms(int ms)
{
	usleep(ms * 1000);
}

//构建语法回调接口
int build_grm_cb(int ecode, const char *info, void *udata)
{
	UserData *grm_data = (UserData *)udata;

	if (NULL != grm_data) {
		grm_data->build_fini = 1;
		grm_data->errcode = ecode;
	}

	if (MSP_SUCCESS == ecode && NULL != info) {
		printf("构建语法成功！ 语法ID:%s\n", info);
		if (NULL != grm_data)
			snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
	}
	else
		printf("构建语法失败！%d\n", ecode);

	return 0;
}

//构建语法
int build_grammar(UserData *udata)
{
	FILE *grm_file                           = NULL;
	char *grm_content                        = NULL;
	unsigned int grm_cnt_len                 = 0;
	char grm_build_params[MAX_PARAMS_LEN]    = {NULL};
	int ret                                  = 0;

	grm_file = fopen(GRM_FILE, "rb");	
	if(NULL == grm_file) {
		printf("打开\"%s\"文件失败！[%s]\n", GRM_FILE, strerror(errno));
		return -1; 
	}

	fseek(grm_file, 0, SEEK_END);
	grm_cnt_len = ftell(grm_file);
	fseek(grm_file, 0, SEEK_SET);

	grm_content = (char *)malloc(grm_cnt_len + 1);
	if (NULL == grm_content)
	{
		printf("内存分配失败!\n");
		fclose(grm_file);
		grm_file = NULL;
		return -1;
	}
	fread((void*)grm_content, 1, grm_cnt_len, grm_file);
	grm_content[grm_cnt_len] = '\0';
	fclose(grm_file);
	grm_file = NULL;

	snprintf(grm_build_params, MAX_PARAMS_LEN - 1, 
		"engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH
		);
	ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

	free(grm_content);
	grm_content = NULL;

	return ret;
}

/* 录音的回调函数 */
static void iat_cb(char *data, unsigned long len, void *user_para)
{
	if(!ros::ok())
	{
		normal_state_flag = false;
	}
	int errcode;
	const char *session_id = (const char *)user_para;

	if(len == 0 || data == NULL)
		return;
	//如果录音成功，则写入语音唤醒缓冲区
	if(!g_is_awaken_succeed){
		errcode = QIVWAudioWrite(session_id, (const void *)data, len, record_state);
	}
	//写入语音唤醒缓冲区成功，否则将关闭录音
	if (MSP_SUCCESS != errcode)
	{
		printf("QIVWAudioWrite failed! error code:%d\n",errcode);
		int ret = stop_record(recorder);
		if (ret != 0) {
			printf("Stop failed! \n");
			//return -E_SR_RECORDFAIL;
		}
		//等待录音结束
		wait_for_rec_stop(recorder, (unsigned int)-1);
		QIVWAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST);//写入最后一块音频
		record_state = MSP_AUDIO_SAMPLE_LAST;
		g_is_awaken_succeed = false;
	}
	//如果写入音频为第一块，则改变标志位
	if(record_state == MSP_AUDIO_SAMPLE_FIRST){
		record_state = MSP_AUDIO_SAMPLE_CONTINUE;
	}
}

//语音唤醒回调函数，当检测到关键词时进入该函数
int cb_ivw_msg_proc( const char *sessionID, int msg, int param1, int param2, const void *info, void *userData )
{
	if(!ros::ok())
	{
		normal_state_flag = false;
	}
	if (MSP_IVW_MSG_ERROR == msg) //唤醒出错消息
	{
		printf("\n\n[唤醒出错消息]  errCode = %d\n\n", param1);
	}
	else if (MSP_IVW_MSG_WAKEUP == msg) //唤醒成功消息
	{
		printf("\n\n[唤醒成功消息]  result = %s\n\n", info);
	}
	else if (MSP_IVW_MSG_ISR_EPS == msg) //唤醒+识别VAD
	{
		//端点检测状态请参考：http://mscdoc.xfyun.cn/windows/api/iFlytekMSCReferenceManual/qisr_8h.html
		char * irs_eps = "";
		switch (param1)
		{
			case MSP_EP_LOOKING_FOR_SPEECH:
				irs_eps = "还没有检测到音频的前端点";
				break;
			case MSP_EP_IN_SPEECH:
				irs_eps = "已经检测到了音频前端点，正在进行正常的音频处理";
				break;
			case MSP_EP_AFTER_SPEECH:
				irs_eps = "检测到音频的后端点，后继的音频会被MSC忽略";
				break;
			case MSP_EP_TIMEOUT:
				irs_eps = "超时";
				break;
			case MSP_EP_ERROR:
				irs_eps = "出现错误";
				break;
			case MSP_EP_MAX_SPEECH:
				irs_eps = "音频过大";
				break;
		}
		printf("\n\n[oneshot vad 端点检测状态]  result = %d(%s)\n\n", param1, irs_eps);
	}
	else if (MSP_IVW_MSG_ISR_RESULT == msg) //oneshot结果消息
	{
		//结果状态值请参考：http://mscdoc.xfyun.cn/windows/api/iFlytekMSCReferenceManual/qisr_8h.html
		char * irs_rsltS = "";
		switch (param1)
		{
		case MSP_REC_STATUS_SUCCESS:
			irs_rsltS = "识别成功";
			break;
		case MSP_REC_STATUS_NO_MATCH:
			irs_rsltS = "识别结束，没有识别结果";
			break;
		case MSP_REC_STATUS_INCOMPLETE:
			irs_rsltS = "正在识别中";
			break;
		case MSP_REC_STATUS_COMPLETE:
			irs_rsltS = "识别结束";
			break;
		}
		printf("\n\n[oneshot结果]  result = \n%s\n\n", info);
		printf("[oneshot结果状态]  isr_status = %d(%s)\n\n", param1, irs_rsltS);
		if (param1 == MSP_REC_STATUS_COMPLETE)
			ISR_STATUS = 1; //识别结束，可以进行sessionend了。
	}
	else 
	{
		printf("\n\nXXXXXXX result = %d\n\n", msg);
	}
	return 0;
}

//语音唤醒主要功能函数
void run_ivw(const char *grammar_list, const char* audio_filename ,  const char* session_begin_params)
{
	const char *session_id = NULL;
	int err_code = MSP_SUCCESS;
	char sse_hints[128];
	long len = 10*FRAME_LEN; //16k音频，10帧 （时长200ms）
	WAVEFORMATEX wavfmt = DEFAULT_FORMAT;
	//开始注册语音唤醒服务
	session_id=QIVWSessionBegin(grammar_list, session_begin_params, &err_code);
	if (err_code != MSP_SUCCESS)
	{
		printf("QIVWSessionBegin failed! error code:%d\n",err_code);
		goto exit;
	}
	//注册回调函数cb_ivw_msg_proc，唤醒结果将在此回调中返回
	err_code = QIVWRegisterNotify(session_id, cb_ivw_msg_proc,NULL);
	if (err_code != MSP_SUCCESS)
	{
		//snprintf(sse_hints, sizeof(sse_hints), "QIVWRegisterNotify errorCode=%d", err_code);
		//printf("QIVWRegisterNotify failed! error code:%d\n",err_code);
		goto exit;
	}
	//创建一个音频流，设置回调函数为iat_cb
	err_code = create_recorder(&recorder, iat_cb, (void*)session_id);
	if (recorder == NULL || err_code != 0) {
			printf("create recorder failed: %d\n", err_code);
			err_code = -E_SR_RECORDFAIL;
			goto exit;
	}

	//打开录音
	err_code = open_recorder(recorder, get_default_input_dev(), &wavfmt);
	if (err_code != 0) {
		printf("recorder open failed: %d\n", err_code);
		err_code = -E_SR_RECORDFAIL;
		goto exit;
	}
	//启动录音
	err_code = start_record(recorder);
	if (err_code != 0) {
		printf("start record failed: %d\n", err_code);
		err_code = -E_SR_RECORDFAIL;
		goto exit;
	}
	record_state = MSP_AUDIO_SAMPLE_FIRST;  //准备开始语音唤醒
	//等待说话，循环进入录音回调函数
	while(record_state != MSP_AUDIO_SAMPLE_LAST)
	{
		if(!normal_state_flag)
		{
			goto exit;
		}
		sleep_ms(200); //模拟人说话时间间隙，10帧的音频时长为200ms
		printf("waiting for awaken%d\n", record_state);
	}
	snprintf(sse_hints, sizeof(sse_hints), "success");

exit:
	if (recorder) {
		if(!is_record_stopped(recorder))
			stop_record(recorder);
		close_recorder(recorder);
		destroy_recorder(recorder);
		recorder = NULL;
	}
	if (NULL != session_id && ISR_STATUS ==1 )
	{
		QIVWSessionEnd(session_id, sse_hints);
	}
}


int main(int argc, char* argv[])
{
	int         ret       = MSP_SUCCESS;
	//const char *lgi_param = "appid = 5c6b6548,work_dir = .";
	//const char *ssb_param = "ivw_threshold=0:1450,sst=wakeup,ivw_res_path =fo|/home/kyle/voice_ws/src/xf_awake/res/ivw/wakeupresource.jet";

	ros::init(argc, argv, "castlex_xiaogu_awake_node");    //初始化节点，向节点管理器注册
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Int32>("/voice/castlex_xiaogu_awake_topic", 1);

	/*
	下面时使用ros::package::getPath函数获取功能包的绝对路径
	使用方法及注意：
	1.#include"ros/package.h"
	2.string path = ros::package::getPath("xf_awake");
	3.在package那里加上：<depend>roslib</depend>
	4.在CmakeLists里的依赖里加上：roslib
	*/
	//下面将获取ssb_param中唤醒模型文件的路径
	string path1 = "ivw_threshold=0:1450,sst=wakeup,ivw_res_path =fo|";
	string path2 = ros::package::getPath("xf_awake");
	//printf(path.data());
	string path3 = "/res/ivw/wakeupresource.jet";
	string my_ssb_param = path1 + path2 + path3;
	string ding_music_path = "/res/music/ding.wav";
	string dong_music_path = "/res/music/dong.wav";
	ding_music_path = string("play ")+path2 + ding_music_path;
	dong_music_path = string("play ")+path2 + dong_music_path;

	ros::NodeHandle nh("~");    //用于launch文件传递参数
	nh.param("appid", my_lgi_param, std::string("appid = 5ddb3b02,work_dir = ."));    //从launch文件获取参数
	
	const char *lgi_param = my_lgi_param.data();
	//const char *ssb_param = my_ssb_param.data();
	char ssb_param[MAX_PARAMS_LEN] = { NULL };
	UserData asr_data;

	ret = MSPLogin(NULL, NULL, lgi_param);
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		goto exit ;//登录失败，退出登录
	}

	/* 构建语法 */
	memset(&asr_data, 0, sizeof(UserData));
	printf("构建离线识别语法网络...\n");
	ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
	if (MSP_SUCCESS != ret) {
		printf("构建语法调用失败！\n");
		goto exit;
	}
	while (1 != asr_data.build_fini)
		sleep_ms(300);
	if (MSP_SUCCESS != asr_data.errcode)
		goto exit;
	printf("离线命令词识别语法网络构建完成，语法id...%s\n", asr_data.grammar_id);

	//oneshot参数设置
	snprintf(ssb_param, MAX_PARAMS_LEN - 1,
		"ivw_threshold=0:1450,sst=oneshot,ivw_res_path =fo|res/ivw/wakeupresource.jet,ivw_shot_word =1, engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, local_grammar = %s, \
		result_type = xml, result_encoding = GB2312,asr_threshold=0",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH,
		asr_data.grammar_id
	);
	
	while(ros::ok())
	{
		run_ivw(NULL, IVW_AUDIO_FILE_NAME, ssb_param); 
		if(g_is_awaken_succeed)
		{
			g_is_awaken_succeed = false;
			std_msgs::Int32 msg;
			system(ding_music_path.data());
			system(dong_music_path.data());
			msg.data = 1;    //将asr返回文本写入消息，发布到topic上
			pub.publish(msg);
		}
	}
exit:
	MSPLogout(); //退出登录
	printf("---------------------------------------------------------------\n");
	printf("Turn off CastleX's voice wakeup feature.....\n");
	printf("---------------------------------------------------------------\n");
	return 0;
}
