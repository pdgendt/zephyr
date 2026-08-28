/*
 *  SPDX-FileCopyrightText: 2026 Basalte bv
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/rtp.h>
#include <zephyr/net/srtp.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "rtp_srtp.h"
#include "srtp_crypto.h"

#define KAT_SSRC 0xcafebabe

/* RFC 3711 Appendix B.2 AES-CM test vectors */
/* clang-format off */
static const uint8_t rfc3711_b2_key[16] = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
};
static const uint8_t rfc3711_b2_iv[16] = {
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0x00, 0x00,
};
static const uint8_t rfc3711_b2_keystream[48] = {
	0xe0, 0x3e, 0xad, 0x09, 0x35, 0xc9, 0x5e, 0x80,
	0xe1, 0x66, 0xb1, 0x6d, 0xd9, 0x2b, 0x4e, 0xb4,
	0xd2, 0x35, 0x13, 0x16, 0x2b, 0x02, 0xd0, 0xf7,
	0x2a, 0x43, 0xa2, 0xfe, 0x4a, 0x5f, 0x97, 0xab,
	0x41, 0xe9, 0x5b, 0x3b, 0xb0, 0xa2, 0xe8, 0xdd,
	0x47, 0x79, 0x01, 0xe4, 0xfc, 0xa8, 0x94, 0xc0,
};
/* clang-format on */

/* RFC 3711 Appendix B.3 key derivation test vectors */
/* clang-format off */
static const uint8_t rfc3711_b3_master_key[16] = {
	0xe1, 0xf9, 0x7a, 0x0d, 0x3e, 0x01, 0x8b, 0xe0,
	0xd6, 0x4f, 0xa3, 0x2c, 0x06, 0xde, 0x41, 0x39,
};
static const uint8_t rfc3711_b3_master_salt[14] = {
	0x0e, 0xc6, 0x75, 0xad, 0x49, 0x8a, 0xfe, 0xeb,
	0xb6, 0x96, 0x0b, 0x3a, 0xab, 0xe6,
};
static const uint8_t rfc3711_b3_cipher_key[16] = {
	0xc6, 0x1e, 0x7a, 0x93, 0x74, 0x4f, 0x39, 0xee,
	0x10, 0x73, 0x4a, 0xfe, 0x3f, 0xf7, 0xa0, 0x87,
};
static const uint8_t rfc3711_b3_auth_key[20] = {
	0xce, 0xbe, 0x32, 0x1f, 0x6f, 0xf7, 0x71, 0x6b,
	0x6f, 0xd4, 0xab, 0x49, 0xaf, 0x25, 0x6a, 0x15,
	0x6d, 0x38, 0xba, 0xa4,
};
static const uint8_t rfc3711_b3_salt[14] = {
	0x30, 0xcb, 0xbc, 0x08, 0x86, 0x3d, 0x8c, 0x85,
	0xd4, 0x9d, 0xb3, 0x4a, 0x9a, 0xe1,
};
/* clang-format on */

/* Known-answer vectors, generated with an independent Python implementation
 * of RFC 3711 / RFC 7714 based on the "cryptography" package.
 */
/* clang-format off */
static const uint8_t kat_master_key[] = {
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
};
static const uint8_t kat_master_salt[] = {
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
};
static const uint8_t kat_rtp[] = {
	0x80, 0x60, 0x12, 0x34, 0x89, 0xab, 0xcd, 0xef,
	0xca, 0xfe, 0xba, 0xbe, 0x50, 0x51, 0x52, 0x53,
	0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
	0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
	0x6c, 0x6d, 0x6e, 0x6f,
};
static const uint8_t kat_srtp_cm_sha1_80[] = {
	0x80, 0x60, 0x12, 0x34, 0x89, 0xab, 0xcd, 0xef,
	0xca, 0xfe, 0xba, 0xbe, 0x39, 0xcc, 0x09, 0x07,
	0xa7, 0x12, 0x89, 0x4a, 0x7e, 0x09, 0xfe, 0x5c,
	0x76, 0x80, 0x7c, 0xcc, 0x32, 0x3d, 0x76, 0x76,
	0x56, 0x2e, 0xb3, 0x67, 0xc4, 0x45, 0xf6, 0xd7,
	0x21, 0xbe, 0x1a, 0x3a, 0xdb, 0x8c, 0xa8, 0xcc,
	0x6f, 0x78, 0x57, 0x33, 0xc5, 0x7c,
};
static const uint8_t kat_srtp_cm_sha1_32[] = {
	0x80, 0x60, 0x12, 0x34, 0x89, 0xab, 0xcd, 0xef,
	0xca, 0xfe, 0xba, 0xbe, 0x39, 0xcc, 0x09, 0x07,
	0xa7, 0x12, 0x89, 0x4a, 0x7e, 0x09, 0xfe, 0x5c,
	0x76, 0x80, 0x7c, 0xcc, 0x32, 0x3d, 0x76, 0x76,
	0x56, 0x2e, 0xb3, 0x67, 0xc4, 0x45, 0xf6, 0xd7,
	0x21, 0xbe, 0x1a, 0x3a, 0xdb, 0x8c, 0xa8, 0xcc,
};
static const uint8_t kat_srtp_gcm[] = {
	0x80, 0x60, 0x12, 0x34, 0x89, 0xab, 0xcd, 0xef,
	0xca, 0xfe, 0xba, 0xbe, 0x64, 0x96, 0x53, 0x5e,
	0x0d, 0x16, 0x3f, 0x42, 0x7e, 0x91, 0x4c, 0x83,
	0xd1, 0x66, 0x53, 0xc4, 0x00, 0xb9, 0xab, 0xcd,
	0xad, 0x58, 0x21, 0x84, 0xcb, 0x05, 0x69, 0xdc,
	0x5d, 0x57, 0xca, 0x4a, 0x96, 0x46, 0xc1, 0x95,
	0xdd, 0x24, 0xb8, 0xb3, 0xe6, 0x4f, 0x40, 0x28,
	0x97, 0x5d, 0x3c, 0x55,
};
static const uint8_t kat_rtp_roc1[] = {
	0x80, 0x60, 0x00, 0x01, 0x89, 0xab, 0xcd, 0xf0,
	0xca, 0xfe, 0xba, 0xbe, 0x50, 0x51, 0x52, 0x53,
	0x54, 0x55, 0x56, 0x57,
};
static const uint8_t kat_srtp_cm_sha1_80_roc1[] = {
	0x80, 0x60, 0x00, 0x01, 0x89, 0xab, 0xcd, 0xf0,
	0xca, 0xfe, 0xba, 0xbe, 0x9b, 0x9b, 0x03, 0xcb,
	0xc5, 0xb4, 0xbb, 0x5d, 0x85, 0x38, 0xab, 0xae,
	0x9d, 0xc0, 0x1f, 0x51, 0xb3, 0x53,
};
/* clang-format on */

static const struct srtp_policy policy_cm_sha1_80 = {
	.cipher = SRTP_CIPHER_AES_128_CM,
	.auth = SRTP_AUTH_HMAC_SHA1_80,
	.master_key = kat_master_key,
	.master_key_len = sizeof(kat_master_key),
	.master_salt = kat_master_salt,
	.master_salt_len = sizeof(kat_master_salt),
};

static const struct srtp_policy policy_cm_sha1_32 = {
	.cipher = SRTP_CIPHER_AES_128_CM,
	.auth = SRTP_AUTH_HMAC_SHA1_32,
	.master_key = kat_master_key,
	.master_key_len = sizeof(kat_master_key),
	.master_salt = kat_master_salt,
	.master_salt_len = sizeof(kat_master_salt),
};

static const struct srtp_policy policy_gcm = {
	.cipher = SRTP_CIPHER_AES_128_GCM,
	.auth = SRTP_AUTH_NULL,
	.master_key = kat_master_key,
	.master_key_len = sizeof(kat_master_key),
	.master_salt = kat_master_salt,
	.master_salt_len = SRTP_AES_GCM_SALT_LEN,
};

static struct srtp_stream tx_stream;
static struct srtp_stream rx_stream;

static void streams_deinit(void *fixture)
{
	ARG_UNUSED(fixture);

	(void)srtp_stream_deinit(&tx_stream);
	(void)srtp_stream_deinit(&rx_stream);
}

/* Protect a small RTP packet with the given sequence number */
static int protect_seq(struct srtp_stream *stream, uint16_t seq, uint8_t *buf, size_t buf_size,
		       size_t *out_len)
{
	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	buf[2] = seq >> 8;
	buf[3] = seq & 0xff;

	return srtp_protect(stream, buf, sizeof(kat_rtp), buf_size, out_len);
}

ZTEST(srtp_tests, test_kdf_rfc3711_b3)
{
	psa_key_id_t master = PSA_KEY_ID_NULL;
	uint8_t k_e[16];
	uint8_t k_a[20];
	uint8_t k_s[14];

	zassert_ok(srtp_crypto_master_key_import(rfc3711_b3_master_key,
						 sizeof(rfc3711_b3_master_key), &master));

	zassert_ok(srtp_crypto_kdf(master, rfc3711_b3_master_salt, SRTP_KDF_LABEL_ENCRYPTION, 0,
				   k_e, sizeof(k_e)));
	zassert_ok(srtp_crypto_kdf(master, rfc3711_b3_master_salt, SRTP_KDF_LABEL_AUTH, 0, k_a,
				   sizeof(k_a)));
	zassert_ok(srtp_crypto_kdf(master, rfc3711_b3_master_salt, SRTP_KDF_LABEL_SALT, 0, k_s,
				   sizeof(k_s)));

	zassert_mem_equal(k_e, rfc3711_b3_cipher_key, sizeof(k_e));
	zassert_mem_equal(k_a, rfc3711_b3_auth_key, sizeof(k_a));
	zassert_mem_equal(k_s, rfc3711_b3_salt, sizeof(k_s));

	srtp_crypto_key_destroy(&master);
}

ZTEST(srtp_tests, test_cm_keystream_rfc3711_b2)
{
	psa_key_id_t key = PSA_KEY_ID_NULL;
	uint8_t data[48] = {0};

	zassert_ok(srtp_crypto_cm_key_import(rfc3711_b2_key, &key));
	zassert_ok(srtp_crypto_cm_crypt(key, rfc3711_b2_iv, data, sizeof(data)));
	zassert_mem_equal(data, rfc3711_b2_keystream, sizeof(data));

	srtp_crypto_key_destroy(&key);
}

ZTEST(srtp_tests, test_protect_kat_cm_sha1_80)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_ok(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len));
	zassert_equal(out_len, sizeof(kat_srtp_cm_sha1_80));
	zassert_mem_equal(buf, kat_srtp_cm_sha1_80, out_len);
}

ZTEST(srtp_tests, test_protect_kat_cm_sha1_32)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_32, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_ok(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len));
	zassert_equal(out_len, sizeof(kat_srtp_cm_sha1_32));
	zassert_mem_equal(buf, kat_srtp_cm_sha1_32, out_len);
}

ZTEST(srtp_tests, test_protect_kat_gcm)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_gcm, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_ok(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len));
	zassert_equal(out_len, sizeof(kat_srtp_gcm));
	zassert_mem_equal(buf, kat_srtp_gcm, out_len);
}

ZTEST(srtp_tests, test_unprotect_kat)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&rx_stream, &policy_cm_sha1_80, KAT_SSRC));

	memcpy(buf, kat_srtp_cm_sha1_80, sizeof(kat_srtp_cm_sha1_80));
	zassert_ok(srtp_unprotect(&rx_stream, buf, sizeof(kat_srtp_cm_sha1_80), &out_len));
	zassert_equal(out_len, sizeof(kat_rtp));
	zassert_mem_equal(buf, kat_rtp, out_len);
}

ZTEST(srtp_tests, test_unprotect_kat_gcm)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&rx_stream, &policy_gcm, KAT_SSRC));

	memcpy(buf, kat_srtp_gcm, sizeof(kat_srtp_gcm));
	zassert_ok(srtp_unprotect(&rx_stream, buf, sizeof(kat_srtp_gcm), &out_len));
	zassert_equal(out_len, sizeof(kat_rtp));
	zassert_mem_equal(buf, kat_rtp, out_len);
}

ZTEST(srtp_tests, test_roc_wrap)
{
	uint8_t buf[128];
	size_t out_len = 0;

	/* TX stream just before a sequence number rollover */
	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));
	tx_stream.roc = 0;
	tx_stream.s_l = 0xfff0;
	tx_stream.seq_initialized = true;
	tx_stream.index_max = 0xfff0;

	memcpy(buf, kat_rtp_roc1, sizeof(kat_rtp_roc1));
	zassert_ok(srtp_protect(&tx_stream, buf, sizeof(kat_rtp_roc1), sizeof(buf), &out_len));
	zassert_equal(tx_stream.roc, 1, "ROC increments across the wrap");
	zassert_equal(out_len, sizeof(kat_srtp_cm_sha1_80_roc1));
	zassert_mem_equal(buf, kat_srtp_cm_sha1_80_roc1, out_len);

	/* RX stream just before the rollover estimates v = ROC + 1 */
	zassert_ok(srtp_stream_init(&rx_stream, &policy_cm_sha1_80, KAT_SSRC));
	rx_stream.roc = 0;
	rx_stream.s_l = 0xfff0;
	rx_stream.seq_initialized = true;
	rx_stream.index_max = 0xfff0;

	zassert_ok(srtp_unprotect(&rx_stream, buf, out_len, &out_len));
	zassert_equal(rx_stream.roc, 1);
	zassert_equal(out_len, sizeof(kat_rtp_roc1));
	zassert_mem_equal(buf, kat_rtp_roc1, out_len);
}

ZTEST(srtp_tests, test_replay)
{
	uint8_t pkt10[128], pkt11[128], pkt12[128], buf[128];
	size_t len10, len11, len12;
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy_cm_sha1_80, KAT_SSRC));

	zassert_ok(protect_seq(&tx_stream, 10, pkt10, sizeof(pkt10), &len10));
	zassert_ok(protect_seq(&tx_stream, 11, pkt11, sizeof(pkt11), &len11));
	zassert_ok(protect_seq(&tx_stream, 12, pkt12, sizeof(pkt12), &len12));

	/* Newest first, then out-of-order within the window */
	memcpy(buf, pkt12, len12);
	zassert_ok(srtp_unprotect(&rx_stream, buf, len12, &out_len));
	memcpy(buf, pkt10, len10);
	zassert_ok(srtp_unprotect(&rx_stream, buf, len10, &out_len));

	/* Replays are rejected */
	memcpy(buf, pkt10, len10);
	zassert_equal(srtp_unprotect(&rx_stream, buf, len10, &out_len), -EALREADY);
	memcpy(buf, pkt12, len12);
	zassert_equal(srtp_unprotect(&rx_stream, buf, len12, &out_len), -EALREADY);

	/* Still in order for the unseen packet */
	memcpy(buf, pkt11, len11);
	zassert_ok(srtp_unprotect(&rx_stream, buf, len11, &out_len));
	memcpy(buf, pkt11, len11);
	zassert_equal(srtp_unprotect(&rx_stream, buf, len11, &out_len), -EALREADY);
}

ZTEST(srtp_tests, test_replay_too_old)
{
	uint8_t pkt_old[128], pkt_new[128], buf[128];
	size_t len_old, len_new;
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy_cm_sha1_80, KAT_SSRC));

	zassert_ok(protect_seq(&tx_stream, 10, pkt_old, sizeof(pkt_old), &len_old));
	zassert_ok(protect_seq(&tx_stream, 10 + CONFIG_SRTP_REPLAY_WINDOW_SIZE + 1, pkt_new,
			       sizeof(pkt_new), &len_new));

	memcpy(buf, pkt_new, len_new);
	zassert_ok(srtp_unprotect(&rx_stream, buf, len_new, &out_len));

	/* The old packet now falls outside the replay window */
	memcpy(buf, pkt_old, len_old);
	zassert_equal(srtp_unprotect(&rx_stream, buf, len_old, &out_len), -EALREADY);
}

ZTEST(srtp_tests, test_tamper)
{
	uint8_t pkt[128], buf[128];
	size_t pkt_len;
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy_cm_sha1_80, KAT_SSRC));

	zassert_ok(protect_seq(&tx_stream, 10, pkt, sizeof(pkt), &pkt_len));

	/* Tampered timestamp (header, offset 4) */
	memcpy(buf, pkt, pkt_len);
	buf[4] ^= 0x01;
	zassert_equal(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len), -EBADMSG);

	/* Tampered payload */
	memcpy(buf, pkt, pkt_len);
	buf[14] ^= 0x01;
	zassert_equal(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len), -EBADMSG);

	/* Tampered tag */
	memcpy(buf, pkt, pkt_len);
	buf[pkt_len - 1] ^= 0x01;
	zassert_equal(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len), -EBADMSG);

	/* Truncated packet */
	memcpy(buf, pkt, pkt_len);
	zassert_equal(srtp_unprotect(&rx_stream, buf, RTP_MIN_HEADER_LEN + 4, &out_len), -EBADMSG);

	/* Failed authentication left the stream state untouched */
	memcpy(buf, pkt, pkt_len);
	zassert_ok(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len));
}

ZTEST(srtp_tests, test_tamper_gcm)
{
	uint8_t pkt[128], buf[128];
	size_t pkt_len;
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_gcm, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy_gcm, KAT_SSRC));

	zassert_ok(protect_seq(&tx_stream, 10, pkt, sizeof(pkt), &pkt_len));

	/* The RTP header is authenticated as additional data */
	memcpy(buf, pkt, pkt_len);
	buf[4] ^= 0x01;
	zassert_equal(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len), -EBADMSG);

	memcpy(buf, pkt, pkt_len);
	buf[14] ^= 0x01;
	zassert_equal(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len), -EBADMSG);

	memcpy(buf, pkt, pkt_len);
	zassert_ok(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len));
}

ZTEST(srtp_tests, test_wrong_key)
{
	static const uint8_t other_key[16] = {0xff, 0xee, 0xdd};
	struct srtp_policy policy = policy_cm_sha1_80;
	uint8_t buf[128];
	size_t out_len = 0;

	policy.master_key = other_key;

	zassert_ok(srtp_stream_init(&rx_stream, &policy, KAT_SSRC));

	memcpy(buf, kat_srtp_cm_sha1_80, sizeof(kat_srtp_cm_sha1_80));
	zassert_equal(srtp_unprotect(&rx_stream, buf, sizeof(kat_srtp_cm_sha1_80), &out_len),
		      -EBADMSG);
}

ZTEST(srtp_tests, test_null_cipher)
{
	struct srtp_policy policy = policy_cm_sha1_80;
	uint8_t buf[128];
	size_t out_len = 0;

	policy.cipher = SRTP_CIPHER_NULL;

	zassert_ok(srtp_stream_init(&tx_stream, &policy, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_ok(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len));
	zassert_equal(out_len, sizeof(kat_rtp) + 10, "Only a tag is appended");
	zassert_mem_equal(buf, kat_rtp, sizeof(kat_rtp), "Payload stays in the clear");

	zassert_ok(srtp_unprotect(&rx_stream, buf, out_len, &out_len));
	zassert_equal(out_len, sizeof(kat_rtp));
	zassert_mem_equal(buf, kat_rtp, out_len);
}

ZTEST(srtp_tests, test_null_null)
{
	struct srtp_policy policy = {
		.cipher = SRTP_CIPHER_NULL,
		.auth = SRTP_AUTH_NULL,
		.allow_null_auth = true,
	};
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_ok(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len));
	zassert_equal(out_len, sizeof(kat_rtp));
	zassert_mem_equal(buf, kat_rtp, out_len);
}

ZTEST(srtp_tests, test_policy_validation)
{
	struct srtp_policy policy;

	/* AEAD ciphers must not configure an auth transform */
	policy = policy_gcm;
	policy.auth = SRTP_AUTH_HMAC_SHA1_80;
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	/* NULL auth needs an explicit opt-in */
	policy = policy_cm_sha1_80;
	policy.auth = SRTP_AUTH_NULL;
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	/* Key and salt sizes */
	policy = policy_cm_sha1_80;
	policy.master_key_len = 15;
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	policy = policy_cm_sha1_80;
	policy.master_salt_len = SRTP_AES_GCM_SALT_LEN;
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	policy = policy_gcm;
	policy.master_salt_len = SRTP_MASTER_SALT_LEN;
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	/* Key derivation rate must be a power of two up to 2^24 */
	policy = policy_cm_sha1_80;
	policy.kdr = 3;
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	policy = policy_cm_sha1_80;
	policy.kdr = BIT(25);
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);

	/* Key material lengths are validated even when no transform needs keys */
	policy = (struct srtp_policy){
		.cipher = SRTP_CIPHER_NULL,
		.auth = SRTP_AUTH_NULL,
		.allow_null_auth = true,
		.master_key = kat_master_key,
		.master_key_len = 32,
		.master_salt = kat_master_salt,
		.master_salt_len = 20,
	};
	zassert_equal(srtp_stream_init(&tx_stream, &policy, KAT_SSRC), -EINVAL);
}

ZTEST(srtp_tests, test_key_expiry)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));
	tx_stream.tx_count = BIT64(48);

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_equal(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len),
		      -EKEYEXPIRED);

	/* Rollover counter exhaustion also expires the key */
	zassert_ok(srtp_stream_init(&rx_stream, &policy_cm_sha1_80, KAT_SSRC));
	rx_stream.roc = UINT32_MAX;
	rx_stream.s_l = 0xfff0;
	rx_stream.seq_initialized = true;
	rx_stream.index_max = ((uint64_t)UINT32_MAX << 16) | 0xfff0;

	zassert_equal(protect_seq(&rx_stream, 1, buf, sizeof(buf), &out_len), -EKEYEXPIRED);
}

ZTEST(srtp_tests, test_kdr_rederivation)
{
	struct srtp_policy policy = policy_cm_sha1_80;
	uint8_t pkt_a[128], pkt_b[128];
	size_t len_a, len_b;
	size_t out_len = 0;

	policy.kdr = BIT(16);

	zassert_ok(srtp_stream_init(&tx_stream, &policy, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy, KAT_SSRC));

	/* One packet in key segment 0, one across the rollover in segment 1 */
	tx_stream.roc = 0;
	tx_stream.s_l = 0xfffe;
	tx_stream.seq_initialized = true;
	tx_stream.index_max = 0xfffe;

	zassert_ok(protect_seq(&tx_stream, 0xffff, pkt_a, sizeof(pkt_a), &len_a));
	zassert_ok(protect_seq(&tx_stream, 0x0000, pkt_b, sizeof(pkt_b), &len_b));
	zassert_equal(tx_stream.key_segment, 1);

	rx_stream.roc = 0;
	rx_stream.s_l = 0xfffe;
	rx_stream.seq_initialized = true;
	rx_stream.index_max = 0xfffe;

	zassert_ok(srtp_unprotect(&rx_stream, pkt_a, len_a, &out_len));
	zassert_mem_equal(pkt_a + 12, kat_rtp + 12, sizeof(kat_rtp) - 12);
	zassert_ok(srtp_unprotect(&rx_stream, pkt_b, len_b, &out_len));
	zassert_mem_equal(pkt_b + 12, kat_rtp + 12, sizeof(kat_rtp) - 12);
	zassert_equal(rx_stream.key_segment, 1);
}

/* Payload lengths that are not multiples of the AES block size exercise the
 * partial-block handling of the cipher backends.
 */
static void short_payload_roundtrip(const struct srtp_policy *policy)
{
	uint8_t buf[64];
	size_t pkt_len = RTP_MIN_HEADER_LEN + 5;
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, policy, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, policy, KAT_SSRC));

	memcpy(buf, kat_rtp, pkt_len);
	zassert_ok(srtp_protect(&tx_stream, buf, pkt_len, sizeof(buf), &out_len));
	zassert_ok(srtp_unprotect(&rx_stream, buf, out_len, &out_len));
	zassert_equal(out_len, pkt_len);
	zassert_mem_equal(buf, kat_rtp, pkt_len);
}

ZTEST(srtp_tests, test_short_payload_cm)
{
	short_payload_roundtrip(&policy_cm_sha1_80);
}

ZTEST(srtp_tests, test_short_payload_gcm)
{
	short_payload_roundtrip(&policy_gcm);
}

ZTEST(srtp_tests, test_kdr_forged_packet_keeps_keys)
{
	struct srtp_policy policy = policy_cm_sha1_80;
	uint8_t pkt[128], buf[128];
	size_t pkt_len;
	size_t out_len = 0;

	policy.kdr = BIT(8);

	zassert_ok(srtp_stream_init(&tx_stream, &policy, KAT_SSRC));
	zassert_ok(srtp_stream_init(&rx_stream, &policy, KAT_SSRC));

	zassert_ok(protect_seq(&tx_stream, 10, pkt, sizeof(pkt), &pkt_len));

	/* A forged packet with a far-future sequence number selects another
	 * key segment; it must fail authentication without mutating the
	 * stream's committed keys or segment.
	 */
	memcpy(buf, pkt, pkt_len);
	buf[2] = 0xf0;
	buf[3] = 0x00;
	zassert_equal(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len), -EBADMSG);
	zassert_equal(rx_stream.key_segment, 0);

	/* The genuine packet still unprotects with the committed keys */
	memcpy(buf, pkt, pkt_len);
	zassert_ok(srtp_unprotect(&rx_stream, buf, pkt_len, &out_len));
	zassert_equal(rx_stream.key_segment, 0);
}

ZTEST(srtp_tests, test_ssrc_mismatch)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	buf[8] ^= 0x01;
	zassert_equal(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len),
		      -EINVAL);
}

ZTEST(srtp_tests, test_no_tag_room)
{
	uint8_t buf[sizeof(kat_rtp) + 4];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_equal(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len),
		      -ENOMEM);
}

ZTEST(srtp_tests, test_deinit_destroys_keys)
{
	uint8_t buf[128];
	size_t out_len = 0;

	zassert_ok(srtp_stream_init(&tx_stream, &policy_cm_sha1_80, KAT_SSRC));
	zassert_ok(srtp_stream_deinit(&tx_stream));

	memcpy(buf, kat_rtp, sizeof(kat_rtp));
	zassert_true(srtp_protect(&tx_stream, buf, sizeof(kat_rtp), sizeof(buf), &out_len) < 0,
		     "Destroyed keys are unusable");
}

ZTEST(srtp_tests, test_session_set_srtp)
{
	static struct rtp_session session;
	static struct srtp_session_ctx ctx;

	memset(&session, 0, sizeof(session));
	session.ssrc = KAT_SSRC;

	zassert_equal(rtp_session_set_srtp(NULL, &policy_cm_sha1_80, NULL, &ctx), -EINVAL);
	zassert_equal(rtp_session_set_srtp(&session, NULL, NULL, &ctx), -EINVAL);
	zassert_equal(rtp_session_set_srtp(&session, &policy_cm_sha1_80, NULL, NULL), -EINVAL);

	zassert_ok(rtp_session_set_srtp(&session, &policy_cm_sha1_80, &policy_cm_sha1_80, &ctx));
	zassert_equal(rtp_session_set_srtp(&session, &policy_cm_sha1_80, NULL, &ctx), -EALREADY);

	zassert_ok(rtp_session_clear_srtp(&session));
	zassert_is_null(session.srtp);

	zassert_ok(rtp_session_set_srtp(&session, &policy_cm_sha1_80, NULL, &ctx));
	zassert_ok(rtp_session_clear_srtp(&session));
}

ZTEST(srtp_tests, test_session_rx_slot_recovery)
{
	static struct rtp_session session;
	static struct srtp_session_ctx ctx;
	uint8_t buf[128];
	size_t out_len = 0;

	memset(&session, 0, sizeof(session));

	zassert_ok(rtp_session_set_srtp(&session, NULL, &policy_cm_sha1_80, &ctx));

	/* Unauthenticated packets with distinct SSRCs must not pin receive
	 * stream slots (changing the SSRC invalidates the tag).
	 */
	for (uint32_t i = 0; i < CONFIG_SRTP_MAX_RX_STREAMS + 2; i++) {
		memcpy(buf, kat_srtp_cm_sha1_80, sizeof(kat_srtp_cm_sha1_80));
		sys_put_be32(0x1000 + i, &buf[8]);
		zassert_equal(
			rtp_srtp_unprotect(&session, buf, sizeof(kat_srtp_cm_sha1_80), &out_len),
			-EBADMSG);
	}

	/* The legitimate source still binds a stream and unprotects */
	memcpy(buf, kat_srtp_cm_sha1_80, sizeof(kat_srtp_cm_sha1_80));
	zassert_ok(rtp_srtp_unprotect(&session, buf, sizeof(kat_srtp_cm_sha1_80), &out_len));
	zassert_equal(out_len, sizeof(kat_rtp));
	zassert_mem_equal(buf, kat_rtp, out_len);

	zassert_ok(rtp_session_clear_srtp(&session));
}

ZTEST_SUITE(srtp_tests, NULL, NULL, NULL, streams_deinit, NULL);
