"""Prints the result of simulator/tests.html from the HTML that headless Edge dumped (run_tests_headless.bat)."""
import html
import re
import sys

text = open(sys.argv[1], encoding="utf-8", errors="replace").read()
title = re.search(r"<title>(.*?)</title>", text, re.S)
print("=" * 70)
print(title.group(1).strip() if title else "no result (did the page load?)")
print("=" * 70)
for block in re.findall(r'<div class="sc">(.*?)</table>', text, re.S):
    name = re.search(r'<span class="tag (PASS|FAIL)">.*?</span>(.*?)<span', block, re.S)
    if not name:
        continue
    print(f"[{name.group(1)}] {html.unescape(re.sub('<[^>]+>', '', name.group(2))).strip()}")
    for fail in re.findall(r'<li class="bad">(.*?)</li>', block, re.S):
        print("      " + html.unescape(re.sub("<[^>]+>", "", fail)))
    for k, v in re.findall(r"<tr><td>(.*?)</td><td>(.*?)</td></tr>", block, re.S):
        print(f"      {html.unescape(k)}: {html.unescape(v)}")
ok = title and "ALL PASS" in title.group(1)
sys.exit(0 if ok else 1)
