#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Оновлює ціни та залишки у price_rozetka_1.xml даними з Хорошопа.

Принцип: файл НЕ перезбирається парсером. Правляться рядково лише три
значення в кожному <offer>, решта байтів лишається недоторканою —
CDATA, форматування, описи, характеристики.

  available="..."          <- наявність у Хорошопі
  <price>...</price>       <- ціна Хорошопа * 1,205, округл. до 5 (half-up)
  <stock_quantity>...</>   <- залишок Хорошопа

Товар, якого немає у фіді Хорошопа, лишається без змін.
Якщо фід недоступний або порожній — скрипт падає, файл не чіпається.
"""

import datetime
import re
import sys
import urllib.request
import xml.etree.ElementTree as ET
from decimal import Decimal, ROUND_HALF_UP

PROM_FEED = "https://techfil.com.ua/content/export/1ea2a9dbded31e6279a51939d1a50079.xml"
TARGET    = "price_rozetka_1.xml"

MARKUP    = "1.205"   # +20,5 % на комісію Розетки
ROUND_TO  = 5         # округлення до найближчих 5 грн
TIMEOUT   = 60


def fetch(url):
    req = urllib.request.Request(url, headers={"User-Agent": "techfil-price-sync/1.0"})
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        return r.read()


def price_for_rozetka(base):
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


OFFER_RE = re.compile(r"<offer\b[^>]*>.*?</offer>", re.S)
URL_RE   = re.compile(r"<url>\s*(.*?)\s*</url>", re.S)
SLUG_RE  = re.compile(r"techfil\.com\.ua/([a-z0-9-]+)/?\s*$", re.I)
AVAIL_RE = re.compile(r'(<offer\b[^>]*?\savailable=")([^"]*)(")')
PRICE_RE = re.compile(r"(<price>)([^<]*)(</price>)")
QTY_RE   = re.compile(r"(<stock_quantity>)([^<]*)(</stock_quantity>)")


def main():
    horoshop = load_horoshop()
    if not horoshop:
        sys.exit("Prom-фід Хорошопа порожній або недоступний — файл не чіпаємо")
    print("Хорошоп: отримано позицій — %d" % len(horoshop))

    with open(TARGET, "r", encoding="utf-8", newline="") as f:
        text = f.read()

    changes, missing = [], []

    def patch_offer(match):
        block = match.group(0)

        m = URL_RE.search(block)
        art = None
        if m:
            s = SLUG_RE.search(m.group(1))
            art = s.group(1).upper() if s else None

        src = horoshop.get(art) if art else None
        if not src:
            missing.append(art or "(без url)")
            return block

        new_price = str(price_for_rozetka(src["price"]))
        new_qty   = str(src["qty"])
        new_avail = "true" if src["available"] else "false"

        pm = PRICE_RE.search(block)
        qm = QTY_RE.search(block)
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
        block = QTY_RE.sub(lambda x: x.group(1) + new_qty + x.group(3), block, count=1)
        return block

    patched = OFFER_RE.sub(patch_offer, text)

    if missing:
        print("Немає у фіді Хорошопа (лишені без змін): %s"
              % ", ".join(sorted(set(missing))))

    if not changes:
        print("Змін немає — файл не переписуємо")
        return

    # позначка свіжості файлу
    stamp = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M")
    patched = re.sub(r'(<yml_catalog[^>]*\bdate=")[^"]*(")',
                     lambda x: x.group(1) + stamp + x.group(2), patched, count=1)

    # страховка: файл має лишитися валідним XML із тією ж кількістю offer
    before_n = len(OFFER_RE.findall(text))
    after_n  = len(OFFER_RE.findall(patched))
    if before_n != after_n:
        sys.exit("Кількість offer змінилася (%d -> %d) — скасовано" % (before_n, after_n))
    try:
        ET.fromstring(patched.encode("utf-8"))
    except ET.ParseError as e:
        sys.exit("Результат не є валідним XML (%s) — скасовано" % e)

    print("Змін: %d" % len(changes))
    for line in changes:
        print("  " + line)

    with open(TARGET, "w", encoding="utf-8", newline="") as f:
        f.write(patched)

    print("Файл %s оновлено" % TARGET)


if __name__ == "__main__":
    main()
