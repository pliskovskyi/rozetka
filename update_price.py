#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Оновлює ціни та залишки у price_rozetka_1.xml і price_prom_1.xml
даними з Хорошопа.

Принцип: файли НЕ перезбираються парсером. Правляться рядково лише три
значення в кожному <offer>, решта байтів лишається недоторканою —
CDATA, форматування, описи, характеристики.

  available="..."             <- наявність у Хорошопі
  <price>...</price>          <- ціна Хорошопа * 1,205, округл. до 5 (half-up)
  <stock_quantity>...</>      <- залишок Хорошопа (файл Розетки)
  <quantity_in_stock>...</>   <- залишок Хорошопа (файл Прому)

Ціни на Промі ті самі, що й на Розетці, тому націнка одна для обох файлів.

Товар, якого немає у фіді Хорошопа, лишається без змін.
Якщо фід недоступний або порожній — скрипт падає, файли не чіпаються.

Атрибут date= у <yml_catalog> оновлюється при КОЖНОМУ запуску, навіть якщо
ціни та залишки не змінилися — інакше Розетка вважає файл ідентичним
попередньому і не обробляє його взагалі.
"""

import datetime
import re
import sys
import urllib.request
import xml.etree.ElementTree as ET
from decimal import Decimal, ROUND_HALF_UP

PROM_FEED = "https://techfil.com.ua/content/export/1ea2a9dbded31e6279a51939d1a50079.xml"

# (файл, тег залишку в цьому файлі)
TARGETS = [
    ("price_rozetka_1.xml", "stock_quantity"),
    ("price_prom_1.xml",    "quantity_in_stock"),
]

MARKUP    = "1.205"   # +20,5 % на комісію Розетки (і та сама ціна на Промі)
ROUND_TO  = 5         # округлення до найближчих 5 грн
TIMEOUT   = 60


def fetch(url):
    req = urllib.request.Request(url, headers={"User-Agent": "techfil-price-sync/1.0"})
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        return r.read()


def price_for_marketplace(base):
    """20,5 % націнки, округлення до найближчих 5 грн, половина — вгору.

    500,00 * 1,205 = 602,50 -> 605   (round() у Python дав би 600)
    """
    v = (Decimal(str(base)) * Decimal(MARKUP) / Decimal(ROUND_TO)
         ).quantize(Decimal("1"), rounding=ROUND_HALF_UP)
    return int(v) * ROUND_TO


def _text(node, tag):
    el = node.find(tag)
    return el.text.strip() if el is not None and el.text else ""


def load_horoshop():
    """{АРТИКУЛ: {'price': float, 'qty': int, 'available': bool}}"""
    root = ET.fromstring(fetch(PROM_FEED))
    out = {}
    for offer in root.iter("offer"):
        art = (_text(offer, "vendorCode") or _text(offer, "article")).upper()
        raw = _text(offer, "price")
        if not art or not raw:
            continue
        try:
            base = float(raw.replace(",", "."))
        except ValueError:
            continue
        try:
            qty = int(float(_text(offer, "quantity_in_stock") or "0"))
        except ValueError:
            qty = 0
        out[art] = {"price": base, "qty": qty,
                    "available": offer.get("available") == "true"}
    return out


OFFER_RE  = re.compile(r"<offer\b[^>]*>.*?</offer>", re.S)
URL_RE    = re.compile(r"<url>\s*(.*?)\s*</url>", re.S)
VENDOR_RE = re.compile(r"<vendorCode>\s*(.*?)\s*</vendorCode>", re.S)
# слаг у кінці URL картки; /ru/ перед слагом (файл Прому) не заважає
SLUG_RE   = re.compile(r"techfil\.com\.ua/(?:ru/)?([a-z0-9-]+)/?\s*$", re.I)
AVAIL_RE  = re.compile(r'(<offer\b[^>]*?\savailable=")([^"]*)(")')
PRICE_RE  = re.compile(r"(<price>)([^<]*)(</price>)")


def article_of(block):
    """Артикул offer: спершу <vendorCode>, інакше — слаг з <url>."""
    m = VENDOR_RE.search(block)
    if m and m.group(1).strip():
        return m.group(1).strip().upper()
    m = URL_RE.search(block)
    if m:
        s = SLUG_RE.search(m.group(1))
        if s:
            return s.group(1).upper()
    return None


def patch_file(target, qty_tag, horoshop, stamp):
    qty_re = re.compile(r"(<%s>)([^<]*)(</%s>)" % (qty_tag, qty_tag))

    with open(target, "r", encoding="utf-8", newline="") as f:
        text = f.read()

    changes, missing = [], []

    def patch_offer(match):
        block = match.group(0)
        art = article_of(block)
        src = horoshop.get(art) if art else None
        if not src:
            missing.append(art or "(без артикула)")
            return block

        new_price = str(price_for_marketplace(src["price"]))
        new_qty   = str(src["qty"])
        new_avail = "true" if src["available"] else "false"

        pm = PRICE_RE.search(block)
        qm = qty_re.search(block)
        am = AVAIL_RE.search(block)
        old_price = pm.group(2).strip() if pm else "-"
        old_qty   = qm.group(2).strip() if qm else "-"
        old_avail = am.group(2) if am else "-"

        if (old_price, old_qty, old_avail) != (new_price, new_qty, new_avail):
            changes.append(
                "%-24s ціна %6s -> %-6s залишок %5s -> %-5s наявність %-5s -> %s"
                % (art, old_price, new_price, old_qty, new_qty, old_avail, new_avail))

        block = AVAIL_RE.sub(lambda x: x.group(1) + new_avail + x.group(3), block, count=1)
        block = PRICE_RE.sub(lambda x: x.group(1) + new_price + x.group(3), block, count=1)
        block = qty_re.sub(lambda x: x.group(1) + new_qty + x.group(3), block, count=1)
        return block

    patched = OFFER_RE.sub(patch_offer, text)

    print("== %s" % target)
    if missing:
        print("Немає у фіді Хорошопа (лишені без змін): %s"
              % ", ".join(sorted(set(missing))))

    if changes:
        print("Змін у цінах/залишках: %d" % len(changes))
        for line in changes:
            print("  " + line)
    else:
        print("Ціни та залишки не змінилися")

    # Позначка свіжості. Оновлюється КОЖНОГО запуску, навіть коли дані ті самі.
    # Розетка порівнює завантажений прайс із попередньою версією і, якщо байти
    # однакові, пропускає обробку цілком («Поточна версія прайс-листа ідентична
    # з попередньою»). Через це будь-яке розходження в кабінеті висіло б доти,
    # доки щось не зміниться в Хорошопі. Свіжий date= гарантує, що кожна
    # синхронізація реально застосується.
    patched = re.sub(r'(<yml_catalog[^>]*\bdate=")[^"]*(")',
                     lambda x: x.group(1) + stamp + x.group(2), patched, count=1)

    # страховка: файл має лишитися валідним XML із тією ж кількістю offer
    before_n = len(OFFER_RE.findall(text))
    after_n  = len(OFFER_RE.findall(patched))
    if before_n != after_n:
        sys.exit("%s: кількість offer змінилася (%d -> %d) — скасовано"
                 % (target, before_n, after_n))
    try:
        ET.fromstring(patched.encode("utf-8"))
    except ET.ParseError as e:
        sys.exit("%s: результат не є валідним XML (%s) — скасовано" % (target, e))

    with open(target, "w", encoding="utf-8", newline="") as f:
        f.write(patched)

    print("Файл %s оновлено" % target)


def main():
    horoshop = load_horoshop()
    if not horoshop:
        sys.exit("Prom-фід Хорошопа порожній або недоступний — файли не чіпаємо")
    print("Хорошоп: отримано позицій — %d" % len(horoshop))

    stamp = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M")
    for target, qty_tag in TARGETS:
        patch_file(target, qty_tag, horoshop, stamp)


if __name__ == "__main__":
    main()
