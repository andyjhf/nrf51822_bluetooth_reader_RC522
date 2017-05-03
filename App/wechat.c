#include "wechat.h"

extern ble_gap_addr_t mac_addr;
char *device_name = "ACR1311 PICC Reader";
uint8_t challenge[4] = {0x01,0x02,0x03,0x04};

void pack_auth_request(uint8_t *buf, uint16_t *length)
{
	AuthRequest *request = (AuthRequest *)malloc(sizeof(AuthRequest));
	request->has_md5_device_type_and_device_id = 0;
	request->proto_version = 65540;
	request->auth_proto = 1;
	request->auth_method = EAM_macNoEncrypt;

	request->has_aes_sign = 0;
	request->has_mac_address = 1;
	
	request->mac_address.len = 6;
	request->mac_address.data = mac_addr.addr;

	request->has_device_name = true;
	request->device_name.str = device_name;
	request->device_name.len = strlen(device_name);

	*length = epb_auth_request_pack_size(request);
	epb_pack_auth_request(request, buf, *length);

	free(request);
}

void unpack_auth_response(uint8_t *buf, uint16_t length)
{
	AuthResponse *response = epb_unpack_auth_response(buf, length);

	if(response != NULL) {
		epb_unpack_auth_response_free(response);
	}
}

void pack_init_request(uint8_t *buf, uint16_t *length)
{
	InitRequest *request = (InitRequest *)malloc(sizeof(InitRequest));
	request->has_challenge = 1;
	request->challenge.data = challenge;
	request->challenge.len = 4;
	*length = epb_init_request_pack_size(request);
	epb_pack_init_request(request, buf, *length);
	free(request);
}

void unpack_init_response(uint8_t *buf, uint16_t length)
{
	InitResponse *response = epb_unpack_init_response(buf, length);

	if(response != NULL) {
		epb_unpack_init_response_free(response);
	}
}





void wechat_pack_request(uint16_t cmdid, uint8_t *buf ,uint8_t *data, uint16_t len)
{
	uint16_t length = 0;
	BpFixHead *mBpFixHead = (BpFixHead *)buf;
	mBpFixHead->bMagicNumber = 0xfe;
	mBpFixHead->bVer = 1;
	mBpFixHead->nCmdId = BigLittleSwap16(cmdid);
	mBpFixHead->nSeq = BigLittleSwap16(1);
	switch(cmdid)
	{
		case ECI_req_auth:
			pack_auth_request((uint8_t *)(mBpFixHead+1),&length);
			mBpFixHead->nLength = BigLittleSwap16(length + sizeof(BpFixHead));
		break;
		case ECI_req_sendData:
			
		break;
		case ECI_req_init:
			pack_init_request((uint8_t *)(mBpFixHead+1),&length);
			mBpFixHead->nLength = BigLittleSwap16(length + sizeof(BpFixHead));
		break;
		case ECI_push_switchView:
			
		break;
		case ECI_push_switchBackgroud:
			
		break;
		case ECI_err_decode:
			
		break;
		default:
			
		break;
	}	
}

int wechat_unpack_response(uint8_t *buf)
{
	BpFixHead *mBpFixHead = (BpFixHead *)buf;
	if(mBpFixHead->bMagicNumber != 0xfe)
		return 1;
	switch(mBpFixHead->nCmdId)
	{
		case ECI_req_auth:
			
		break;
		case ECI_req_sendData:
			
		break;
		case ECI_req_init:
			
		break;
		case ECI_resp_auth:
			
		break;
		case ECI_resp_sendData:
			
		break;
		case ECI_resp_init:
			
		break;
		case ECI_push_recvData:
			
		break;
		case ECI_push_switchView:
			
		break;
		case ECI_push_switchBackgroud:
			
		break;
		case ECI_err_decode:
			
		break;
		default:
			
		break;
	}	
	return 0;
}


