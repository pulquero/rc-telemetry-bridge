/**
 * Utilities to do Signature Version 4.
 * @class SigV4Utils
 */
function SigV4Utils() {}

SigV4Utils.getSignatureKey = function (key, date, region, service) {
	const kDate = AWS.util.crypto.hmac('AWS4' + key, date, 'buffer');
	const kRegion = AWS.util.crypto.hmac(kDate, region, 'buffer');
	const kService = AWS.util.crypto.hmac(kRegion, service, 'buffer');
	const kCredentials = AWS.util.crypto.hmac(kService, 'aws4_request', 'buffer');
	return kCredentials;
};

SigV4Utils.getSignedIoTUrl = function(host, config) {
	const region = config.region;
	const credentials = config.credentials;
	const datetime = AWS.util.date.iso8601(new Date()).replace(/[:\-]|\.\d{3}/g, '');
	const date = datetime.substr(0, 8);

	const method = 'GET';
	const protocol = 'wss';
	const uri = '/mqtt';
	const service = 'iotdevicegateway';
	const algorithm = 'AWS4-HMAC-SHA256';

	const credentialScope = date + '/' + region + '/' + service + '/' + 'aws4_request';
	var canonicalQuerystring = 'X-Amz-Algorithm=' + algorithm;
	canonicalQuerystring += '&X-Amz-Credential=' + encodeURIComponent(credentials.accessKeyId + '/' + credentialScope);
	canonicalQuerystring += '&X-Amz-Date=' + datetime;
	canonicalQuerystring += '&X-Amz-SignedHeaders=host';

	const canonicalHeaders = 'host:' + host + '\n';
	const payloadHash = AWS.util.crypto.sha256('', 'hex');
	const canonicalRequest = method + '\n' + uri + '\n' + canonicalQuerystring + '\n' + canonicalHeaders + '\nhost\n' + payloadHash;

	const stringToSign = algorithm + '\n' + datetime + '\n' + credentialScope + '\n' + AWS.util.crypto.sha256(canonicalRequest, 'hex');
	const signingKey = SigV4Utils.getSignatureKey(credentials.secretAccessKey, date, region, service);
	const signature = AWS.util.crypto.hmac(signingKey, stringToSign, 'hex');

	canonicalQuerystring += '&X-Amz-Signature=' + signature;
	if (credentials.sessionToken) {
		canonicalQuerystring += '&X-Amz-Security-Token=' + encodeURIComponent(credentials.sessionToken);
	}

	const requestUrl = protocol + '://' + host + uri + '?' + canonicalQuerystring;
	return requestUrl;
};
