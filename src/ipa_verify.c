/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa_module.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc != 2) { fprintf(stderr, "Usage: ccamera-ipa-verify MODULE\n"); return 2; }
    cc_ipa_module *module = NULL;
    int ret = cc_ipa_module_open(argv[1], &module);
    cc_ipa_signature_status status = CC_IPA_SIGNATURE_UNAVAILABLE;
    if (!ret) ret = cc_ipa_module_verify_signature(module, &status);
    cc_ipa_module_close(module);
    if (ret < 0) { fprintf(stderr, "%s: %s\n", argv[1], strerror(-ret)); return 2; }
    const char *names[] = {"valid", "missing", "invalid", "unavailable"};
    printf("%s: %s\n", argv[1], names[status]);
    return status == CC_IPA_SIGNATURE_VALID ? 0 : status == CC_IPA_SIGNATURE_UNAVAILABLE ? 2 : 1;
}
