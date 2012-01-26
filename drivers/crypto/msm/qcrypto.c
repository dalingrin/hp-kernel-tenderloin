/* Qualcomm Crypto driver
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/crypto.h>
#include <linux/kernel.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>

#include <crypto/ctr.h>
#include <crypto/des.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/scatterwalk.h>

#include "inc/qce.h"

#define MAX_CRYPTO_DEVICE 3
#define DEBUG_MAX_FNAME  16
#define DEBUG_MAX_RW_BUF 1024

struct crypto_stat {
	u32 aead_sha1_aes_enc;
	u32 aead_sha1_aes_dec;
	u32 aead_sha1_des_enc;
	u32 aead_sha1_des_dec;
	u32 aead_sha1_3des_enc;
	u32 aead_sha1_3des_dec;
	u32 aead_op_success;
	u32 aead_op_fail;
	u32 ablk_cipher_aes_enc;
	u32 ablk_cipher_aes_dec;
	u32 ablk_cipher_des_enc;
	u32 ablk_cipher_des_dec;
	u32 ablk_cipher_3des_enc;
	u32 ablk_cipher_3des_dec;
	u32 ablk_cipher_op_success;
	u32 ablk_cipher_op_fail;
};
static struct crypto_stat _qcrypto_stat[MAX_CRYPTO_DEVICE];
static struct dentry *_debug_dent;
static char _debug_read_buf[DEBUG_MAX_RW_BUF];

struct crypto_priv {
	/* the lock protects queue and req*/
	spinlock_t lock;

	/* qce handle */
	void *qce;

	/* list of  registered algorithms */
	struct list_head alg_list;

	/* platform device */
	struct platform_device *pdev;

	/* current active request */
	struct crypto_async_request *req;
	int res;

	/* request queue */
	struct crypto_queue queue;

	struct tasklet_struct done_tasklet;
};

#define QCRYPTO_MAX_KEY_SIZE	64

/* max of AES_BLOCK_SIZE, DES3_EDE_BLOCK_SIZE */
#define QCRYPTO_MAX_IV_LENGTH	16

struct qcrypto_ctx {
	u8 auth_key[QCRYPTO_MAX_KEY_SIZE];
	u8 iv[QCRYPTO_MAX_IV_LENGTH];

	u8 enc_key[QCRYPTO_MAX_KEY_SIZE];
	unsigned int enc_key_len;

	unsigned int authsize;
	unsigned int auth_key_len;

	struct crypto_priv *cp;
};

struct qcrypto_request_ctx {
	u8 *iv;
	unsigned int ivsize;
	int  aead;
	enum qce_cipher_alg_enum alg;
	enum qce_cipher_dir_enum dir;
	enum qce_cipher_mode_enum mode;
};


struct qcrypto_alg {
	struct list_head entry;
	struct crypto_alg crypto_alg;
	struct crypto_priv *cp;
};

static void _start_qcrypto_process(struct crypto_priv *cp);

static struct qcrypto_alg *_qcrypto_alg_alloc(struct crypto_priv *cp,
		struct crypto_alg *template)
{
	struct qcrypto_alg *q_alg;

	q_alg = kzalloc(sizeof(struct qcrypto_alg), GFP_KERNEL);
	if (!q_alg)
		return ERR_PTR(-ENOMEM);

	q_alg->crypto_alg = *template;
	q_alg->cp = cp;

	return q_alg;
};

static int _qcrypto_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct qcrypto_alg *q_alg;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(tfm);

	q_alg = container_of(alg, struct qcrypto_alg, crypto_alg);

	/* update context with ptr to cp */
	ctx->cp = q_alg->cp;

	/* random first IV */
	get_random_bytes(ctx->iv, QCRYPTO_MAX_IV_LENGTH);

	return 0;
};

static int _qcrypto_cra_ablkcipher_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct qcrypto_request_ctx);
	return _qcrypto_cra_init(tfm);
};

static int _qcrypto_cra_aead_init(struct crypto_tfm *tfm)
{
	tfm->crt_aead.reqsize = sizeof(struct qcrypto_request_ctx);
	return _qcrypto_cra_init(tfm);
};

static int _disp_stats(int id)
{
	struct crypto_stat *pstat;
	int len = 0;

	pstat = &_qcrypto_stat[id];
	len = snprintf(_debug_read_buf, DEBUG_MAX_RW_BUF - 1,
			"\nQualcomm crypto accelerator %d Statistics:\n",
				id + 1);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK AES CIPHER encryption   : %d\n",
					pstat->ablk_cipher_aes_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK AES CIPHER decryption   : %d\n",
					pstat->ablk_cipher_aes_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK DES CIPHER encryption   : %d\n",
					pstat->ablk_cipher_des_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK DES CIPHER decryption   : %d\n",
					pstat->ablk_cipher_des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK 3DES CIPHER encryption  : %d\n",
					pstat->ablk_cipher_3des_enc);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK 3DES CIPHER decryption  : %d\n",
					pstat->ablk_cipher_3des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK CIPHER operation success: %d\n",
					pstat->ablk_cipher_op_success);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK CIPHER operation fail   : %d\n",
					pstat->ablk_cipher_op_fail);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-AES encryption      : %d\n",
					pstat->aead_sha1_aes_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-AES decryption      : %d\n",
					pstat->aead_sha1_aes_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-DES encryption      : %d\n",
					pstat->aead_sha1_des_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-DES decryption      : %d\n",
					pstat->aead_sha1_des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-3DES encryption     : %d\n",
					pstat->aead_sha1_3des_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-3DES decryption     : %d\n",
					pstat->aead_sha1_3des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD operation success       : %d\n",
					pstat->aead_op_success);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD operation fail          : %d\n",
					pstat->aead_op_fail);

	return len;
}

static int _qcrypto_remove(struct platform_device *pdev)
{
	struct crypto_priv *cp;
	struct qcrypto_alg *q_alg;
	struct qcrypto_alg *n;

	cp = platform_get_drvdata(pdev);

	if (!cp)
		return 0;

	list_for_each_entry_safe(q_alg, n, &cp->alg_list, entry) {
		crypto_unregister_alg(&q_alg->crypto_alg);
		list_del(&q_alg->entry);
		kfree(q_alg);
	}

	if (cp->qce)
		qce_close(cp->qce);
	tasklet_kill(&cp->done_tasklet);
	kfree(cp);
	return 0;
};

static int _qcrypto_setkey_aes(struct crypto_ablkcipher *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(tfm);

	switch (len) {
	case AES_KEYSIZE_128:
	case AES_KEYSIZE_192:
	case AES_KEYSIZE_256:
		break;
	default:
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	};
	ctx->enc_key_len = len;
	memcpy(ctx->enc_key, key, len);
	return 0;
};

static int _qcrypto_setkey_des(struct crypto_ablkcipher *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(tfm);
	u32 tmp[DES_EXPKEY_WORDS];
	int ret = des_ekey(tmp, key);

	if (len != DES_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	};

	if (unlikely(ret == 0) && (tfm->crt_flags & CRYPTO_TFM_REQ_WEAK_KEY)) {
		tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
		return -EINVAL;
	}

	ctx->enc_key_len = len;
	memcpy(ctx->enc_key, key, len);
	return 0;
};

static int _qcrypto_setkey_3des(struct crypto_ablkcipher *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(tfm);

	if (len != DES3_EDE_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	};
	ctx->enc_key_len = len;
	memcpy(ctx->enc_key, key, len);
	return 0;
};

static void req_done(unsigned long data)
{
	struct crypto_async_request *areq;
	struct crypto_priv *cp = (struct crypto_priv *)data;
	unsigned long flags;

	spin_lock_irqsave(&cp->lock, flags);
	areq = cp->req;
	cp->req = NULL;
	spin_unlock_irqrestore(&cp->lock, flags);

	if (areq)
		areq->complete(areq, cp->res);
	_start_qcrypto_process(cp);
};

static void _qce_ablk_cipher_complete(void *cookie, unsigned char *icb,
		unsigned char *iv, int ret)
{
	struct ablkcipher_request *areq = (struct ablkcipher_request *) cookie;
	struct crypto_ablkcipher *ablk = crypto_ablkcipher_reqtfm(areq);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qce_ablk_cipher_complete: %p ret %d\n",
				areq, ret);
#endif
	if (iv)
		memcpy(ctx->iv, iv, crypto_ablkcipher_ivsize(ablk));

	if (ret) {
		cp->res = -ENXIO;
		pstat->ablk_cipher_op_fail++;
	} else {
		cp->res = 0;
		pstat->ablk_cipher_op_success++;
	}

	tasklet_schedule(&cp->done_tasklet);
};


static void _qce_aead_complete(void *cookie, unsigned char *icv,
				unsigned char *iv, int ret)
{
	struct aead_request *areq = (struct aead_request *) cookie;
	struct crypto_aead *aead = crypto_aead_reqtfm(areq);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_request_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	if (ret == 0) {
		rctx = aead_request_ctx(areq);
		if (rctx->dir  == QCE_ENCRYPT) {
			/* copy the icv to dst */
			scatterwalk_map_and_copy(icv, areq->dst,
					areq->cryptlen,
					ctx->authsize, 1);

		} else {
			unsigned char tmp[SHA256_DIGESTSIZE];

			/* compare icv from src */
			scatterwalk_map_and_copy(tmp,
					areq->src, areq->cryptlen -
					ctx->authsize, ctx->authsize,
					0);
			ret = memcmp(icv, tmp, ctx->authsize);
			if (ret != 0)
				ret = -EBADMSG;

		}
	} else {
		ret = -ENXIO;
	}

	if (iv)
		memcpy(ctx->iv, iv, crypto_aead_ivsize(aead));

	if (ret)
		pstat->aead_op_fail++;
	else
		pstat->aead_op_success++;

	tasklet_schedule(&cp->done_tasklet);
}

static void _start_qcrypto_process(struct crypto_priv *cp)
{
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;
	unsigned long flags;
	u32 type;
	struct qce_req qreq;
	int ret;
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

again:
	spin_lock_irqsave(&cp->lock, flags);
	if (cp->req == NULL) {
		backlog = crypto_get_backlog(&cp->queue);
		async_req = crypto_dequeue_request(&cp->queue);
		cp->req = async_req;
	}
	spin_unlock_irqrestore(&cp->lock, flags);
	if (!async_req)
		return;
	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);
	type = crypto_tfm_alg_type(async_req->tfm);
	if (type == CRYPTO_ALG_TYPE_ABLKCIPHER) {
		struct ablkcipher_request *req;
		struct crypto_ablkcipher *tfm;

		req = container_of(async_req, struct ablkcipher_request, base);
		ctx = crypto_tfm_ctx(async_req->tfm);
		rctx = ablkcipher_request_ctx(req);
		tfm = crypto_ablkcipher_reqtfm(req);

		qreq.op = QCE_REQ_ABLK_CIPHER;
		qreq.qce_cb = _qce_ablk_cipher_complete;
		qreq.areq = req;
		qreq.alg = rctx->alg;
		qreq.dir = rctx->dir;
		qreq.mode = rctx->mode;
		qreq.enckey = ctx->enc_key;
		qreq.encklen = ctx->enc_key_len;
		qreq.iv = req->info;
		qreq.ivsize = crypto_ablkcipher_ivsize(tfm);
		qreq.cryptlen = req->nbytes;
		qreq.use_pmem = 0;
		ret =  qce_ablk_cipher_req(cp->qce, &qreq);

	} else {
		struct aead_request *req;

		req = container_of(async_req, struct aead_request, base);
		rctx = aead_request_ctx(req);
		ctx = crypto_tfm_ctx(async_req->tfm);

		qreq.op = QCE_REQ_AEAD;
		qreq.qce_cb = _qce_aead_complete;

		qreq.areq = req;
		qreq.alg = rctx->alg;
		qreq.dir = rctx->dir;
		qreq.mode = rctx->mode;
		qreq.iv = rctx->iv;

		qreq.enckey = ctx->enc_key;
		qreq.encklen = ctx->enc_key_len;
		qreq.authkey = ctx->auth_key;
		qreq.authklen = ctx->auth_key_len;

		ret =  qce_aead_req(cp->qce, &qreq);
	};

	if (ret) {

		spin_lock_irqsave(&cp->lock, flags);
		cp->req = NULL;
		spin_unlock_irqrestore(&cp->lock, flags);

		if (type == CRYPTO_ALG_TYPE_ABLKCIPHER)
			pstat->ablk_cipher_op_fail++;
		else
			pstat->aead_op_fail++;

		async_req->complete(async_req, ret);
		goto again;
	};
};

static int _qcrypto_queue_req(struct crypto_priv *cp,
				struct crypto_async_request *req)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&cp->lock, flags);
	ret = crypto_enqueue_request(&cp->queue, req);
	spin_unlock_irqrestore(&cp->lock, flags);
	_start_qcrypto_process(cp);
	return ret;
}

static int _qcrypto_enc_aes_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_enc_aes_ecb: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_aes_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_enc_aes_cbc: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_aes_ctr(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
				CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_enc_aes_ctr: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CTR;

	pstat->ablk_cipher_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_3des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_3des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_3des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_3des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_aes_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
				CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_dec_aes_ecb: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_aes_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
				CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_dec_aes_cbc: %p\n", req);
#endif

	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_aes_ctr(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_dec_aes_ctr: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->mode = QCE_MODE_CTR;

	/* Note. There is no such thing as aes/counter mode, decrypt */
	rctx->dir = QCE_ENCRYPT;

	pstat->ablk_cipher_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_3des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_3des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_3des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_3des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};


static int _qcrypto_aead_setauthsize(struct crypto_aead *authenc,
				unsigned int authsize)
{
	struct qcrypto_ctx *ctx = crypto_aead_ctx(authenc);

	ctx->authsize = authsize;
	return 0;
}

static int _qcrypto_aead_setkey(struct crypto_aead *tfm, const u8 *key,
			unsigned int keylen)
{
	struct qcrypto_ctx *ctx = crypto_aead_ctx(tfm);
	struct rtattr *rta = (struct rtattr *)key;
	struct crypto_authenc_key_param *param;

	if (!RTA_OK(rta, keylen))
		goto badkey;
	if (rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM)
		goto badkey;
	if (RTA_PAYLOAD(rta) < sizeof(*param))
		goto badkey;

	param = RTA_DATA(rta);
	ctx->enc_key_len = be32_to_cpu(param->enckeylen);

	key += RTA_ALIGN(rta->rta_len);
	keylen -= RTA_ALIGN(rta->rta_len);

	if (keylen < ctx->enc_key_len)
		goto badkey;

	ctx->auth_key_len = keylen - ctx->enc_key_len;
	if (ctx->enc_key_len >= QCRYPTO_MAX_KEY_SIZE ||
				ctx->auth_key_len >= QCRYPTO_MAX_KEY_SIZE)
		goto badkey;
	memset(ctx->auth_key, 0, QCRYPTO_MAX_KEY_SIZE);
	memcpy(ctx->enc_key, key + ctx->auth_key_len, ctx->enc_key_len);
	memcpy(ctx->auth_key, key, ctx->auth_key_len);

	return 0;
badkey:
	ctx->enc_key_len = 0;
	crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
	return -EINVAL;
}

static int _qcrypto_aead_encrypt_aes_cbc(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_aead_encrypt_aes_cbc: %p\n", req);
#endif

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_aes_cbc(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_aead_decrypt_aes_cbc: %p\n", req);
#endif
	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_aes_cbc(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_request_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
}

#ifdef QCRYPTO_AEAD_AES_CTR
static int _qcrypto_aead_encrypt_aes_ctr(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CTR;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_aes_ctr(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;

	/* Note. There is no such thing as aes/counter mode, decrypt */
	rctx->dir = QCE_ENCRYPT;

	rctx->mode = QCE_MODE_CTR;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_aes_ctr(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_request_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CTR;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
};
#endif /* QCRYPTO_AEAD_AES_CTR */

static int _qcrypto_aead_encrypt_des_cbc(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_des_cbc(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_des_cbc(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_request_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_des_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
}

static int _qcrypto_aead_encrypt_3des_cbc(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_3des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_3des_cbc(struct aead_request *req)
{
	struct qcrypto_request_ctx *rctx;
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_3des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_3des_cbc(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_request_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_3des_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
}

static struct crypto_alg _qcrypto_algos[] = {
	{
		.cra_name		= "ecb(aes)",
		.cra_driver_name	= "qcrypto-ecb-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey		= _qcrypto_setkey_aes,
				.encrypt	= _qcrypto_enc_aes_ecb,
				.decrypt	= _qcrypto_dec_aes_ecb,
			},
		},
	},
	{
		.cra_name	= "cbc(aes)",
		.cra_driver_name = "qcrypto-cbc-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey		= _qcrypto_setkey_aes,
				.encrypt	= _qcrypto_enc_aes_cbc,
				.decrypt	= _qcrypto_dec_aes_cbc,
			},
		},
	},
	{
		.cra_name	= "ctr(aes)",
		.cra_driver_name = "qcrypto-ctr-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey		= _qcrypto_setkey_aes,
				.encrypt	= _qcrypto_enc_aes_ctr,
				.decrypt	= _qcrypto_dec_aes_ctr,
			},
		},
	},
	{
		.cra_name		= "ecb(des)",
		.cra_driver_name	= "qcrypto-ecb-des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey		= _qcrypto_setkey_des,
				.encrypt	= _qcrypto_enc_des_ecb,
				.decrypt	= _qcrypto_dec_des_ecb,
			},
		},
	},
	{
		.cra_name	= "cbc(des)",
		.cra_driver_name = "qcrypto-cbc-des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey		= _qcrypto_setkey_des,
				.encrypt	= _qcrypto_enc_des_cbc,
				.decrypt	= _qcrypto_dec_des_cbc,
			},
		},
	},
	{
		.cra_name		= "ecb(des3_ede)",
		.cra_driver_name	= "qcrypto-ecb-3des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.min_keysize	= DES3_EDE_KEY_SIZE,
				.max_keysize	= DES3_EDE_KEY_SIZE,
				.setkey		= _qcrypto_setkey_3des,
				.encrypt	= _qcrypto_enc_3des_ecb,
				.decrypt	= _qcrypto_dec_3des_ecb,
			},
		},
	},
	{
		.cra_name	= "cbc(des3_ede)",
		.cra_driver_name = "qcrypto-cbc-3des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= DES3_EDE_BLOCK_SIZE,
				.min_keysize	= DES3_EDE_KEY_SIZE,
				.max_keysize	= DES3_EDE_KEY_SIZE,
				.setkey		= _qcrypto_setkey_3des,
				.encrypt	= _qcrypto_enc_3des_cbc,
				.decrypt	= _qcrypto_dec_3des_cbc,
			},
		},
	},
	{
		.cra_name	= "authenc(hmac(sha1),cbc(aes))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = AES_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_aes_cbc,
				.decrypt = _qcrypto_aead_decrypt_aes_cbc,
				.givencrypt = _qcrypto_aead_givencrypt_aes_cbc,
				.geniv = "<built-in>",
			}
		}
	},

#ifdef QCRYPTO_AEAD_AES_CTR
	{
		.cra_name	= "authenc(hmac(sha1),ctr(aes))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-ctr-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = AES_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_aes_ctr,
				.decrypt = _qcrypto_aead_decrypt_aes_ctr,
				.givencrypt = _qcrypto_aead_givencrypt_aes_ctr,
				.geniv = "<built-in>",
			}
		}
	},
#endif /* QCRYPTO_AEAD_AES_CTR */
	{
		.cra_name	= "authenc(hmac(sha1),cbc(des))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = DES_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_des_cbc,
				.decrypt = _qcrypto_aead_decrypt_des_cbc,
				.givencrypt = _qcrypto_aead_givencrypt_des_cbc,
				.geniv = "<built-in>",
			}
		}
	},
	{
		.cra_name	= "authenc(hmac(sha1),cbc(des3_ede))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-3des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = DES3_EDE_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_3des_cbc,
				.decrypt = _qcrypto_aead_decrypt_3des_cbc,
				.givencrypt = _qcrypto_aead_givencrypt_3des_cbc,
				.geniv = "<built-in>",
			}
		}
	},
};

static int  _qcrypto_probe(struct platform_device *pdev)
{
	int rc = 0;
	void *handle;
	struct crypto_priv *cp;
	int i;

	if (pdev->id >= MAX_CRYPTO_DEVICE) {
		printk(KERN_ERR "%s: device id %d  exceeds allowed %d\n",
				__func__, pdev->id, MAX_CRYPTO_DEVICE);
		return -ENOENT;
	}

	cp = kzalloc(sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	/* open qce */
	handle = qce_open(pdev, &rc);
	if (handle == NULL) {
		kfree(cp);
		platform_set_drvdata(pdev, NULL);
		return rc;
	}

	INIT_LIST_HEAD(&cp->alg_list);
	platform_set_drvdata(pdev, cp);
	spin_lock_init(&cp->lock);
	tasklet_init(&cp->done_tasklet, req_done, (unsigned long)cp);
	crypto_init_queue(&cp->queue, 50);
	cp->qce = handle;
	cp->pdev = pdev;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(_qcrypto_algos); i++) {
		struct qcrypto_alg *q_alg;

		q_alg = _qcrypto_alg_alloc(cp, &_qcrypto_algos[i]);
		if (IS_ERR(q_alg)) {
			rc = PTR_ERR(q_alg);
			goto err;
		}

		if (((q_alg->crypto_alg.cra_flags & CRYPTO_ALG_TYPE_MASK) ==
				CRYPTO_ALG_TYPE_AEAD) &&
					qce_hmac_support(handle) <= 0) {
			kfree(q_alg);
			continue;
		}

		rc = crypto_register_alg(&q_alg->crypto_alg);
		if (rc) {
			dev_err(&pdev->dev, "%s alg registration failed\n",
					q_alg->crypto_alg.cra_driver_name);
			kfree(q_alg);
		} else {
			list_add_tail(&q_alg->entry, &cp->alg_list);
			dev_info(&pdev->dev, "%s\n",
					q_alg->crypto_alg.cra_driver_name);
		}
	}
	return 0;
err:
	_qcrypto_remove(pdev);
	return rc;
};

static struct platform_driver _qualcomm_crypto = {
	.probe          = _qcrypto_probe,
	.remove         = _qcrypto_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "qcrypto",
	},
};

static int _debug_qcrypto[MAX_CRYPTO_DEVICE];

static int _debug_stats_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t _debug_stats_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int rc = -EINVAL;
	int qcrypto = *((int *) file->private_data);
	int len;

	len = _disp_stats(qcrypto);

	rc = simple_read_from_buffer((void __user *) buf, len,
			ppos, (void *) _debug_read_buf, len);

	return rc;
}

static ssize_t _debug_stats_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{

	int qcrypto = *((int *) file->private_data);

	memset((char *)&_qcrypto_stat[qcrypto], 0, sizeof(struct crypto_stat));
	return count;
};

static const struct file_operations _debug_stats_ops = {
	.open =         _debug_stats_open,
	.read =         _debug_stats_read,
	.write =        _debug_stats_write,
};

static int _qcrypto_debug_init(void)
{
	int rc;
	char name[DEBUG_MAX_FNAME];
	int i;
	struct dentry *dent;

	_debug_dent = debugfs_create_dir("qcrypto", NULL);
	if (IS_ERR(_debug_dent)) {
		pr_err("qcrypto debugfs_create_dir fail, error %ld\n",
				PTR_ERR(_debug_dent));
		return PTR_ERR(_debug_dent);
	}

	for (i = 0; i < MAX_CRYPTO_DEVICE; i++) {
		snprintf(name, DEBUG_MAX_FNAME-1, "stats-%d", i+1);
		_debug_qcrypto[i] = i;
		dent = debugfs_create_file(name, 0644, _debug_dent,
				&_debug_qcrypto[i], &_debug_stats_ops);
		if (dent == NULL) {
			pr_err("qcrypto debugfs_create_file fail, error %ld\n",
					PTR_ERR(dent));
			rc = PTR_ERR(dent);
			goto err;
		}
	}
	return 0;
err:
	debugfs_remove_recursive(_debug_dent);
	return rc;
}

static int __init _qcrypto_init(void)
{
	int rc;

	rc = _qcrypto_debug_init();
	if (rc)
		return rc;

	return platform_driver_register(&_qualcomm_crypto);
}

static void __exit _qcrypto_exit(void)
{
	printk(KERN_ALERT "%s Unregister QCRYPTO\n", __func__);
	debugfs_remove_recursive(_debug_dent);
	platform_driver_unregister(&_qualcomm_crypto);
}

module_init(_qcrypto_init);
module_exit(_qcrypto_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mona Hossain <mhossain@codeaurora.org>");
MODULE_DESCRIPTION("Qualcomm Crypto driver");
MODULE_VERSION("1.01");
