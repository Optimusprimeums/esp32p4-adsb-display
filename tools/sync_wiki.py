#!/usr/bin/env python3
"""
Mirror docs/*.md into a checked-out GitHub wiki repository.

  git clone https://github.com/<owner>/<repo>.wiki.git ../wiki
  python tools/sync_wiki.py ../wiki
  cd ../wiki && git add -A && git commit -m "Sync from docs" && git push

The docs folder (published with MkDocs) stays the source of truth.
"""
import os
import re
import sys

import yaml  # pip install pyyaml

ROOT = os.path.join(os.path.dirname(__file__), '..')


def page_name(md_file):
    """docs file name -> wiki page name (Home for index.md)."""
    base = os.path.splitext(os.path.basename(md_file))[0]
    return 'Home' if base == 'index' else base


def convert(text):
    # links to other pages: foo.md#bar -> foo#bar, index.md -> Home
    def fix(m):
        target, anchor = m.group(2), m.group(3) or ''
        return '](%s%s)' % (page_name(target), anchor)
    text = re.sub(r'\]\(((?!https?:)([\w\-]+\.md))(#[^)]*)?\)', lambda m: fix(m), text)
    # MkDocs admonitions -> blockquotes
    out, lines, i = [], text.split('\n'), 0
    while i < len(lines):
        m = re.match(r'^!!! (\w+)(?: "(.*)")?\s*$', lines[i])
        if m:
            title = m.group(2) or m.group(1).capitalize()
            out.append('> **%s**' % title)
            out.append('>')
            i += 1
            while i < len(lines) and (lines[i].startswith('    ') or lines[i] == ''):
                if lines[i] == '' and (i + 1 >= len(lines) or not lines[i + 1].startswith('    ')):
                    break
                out.append('> ' + lines[i][4:])
                i += 1
            continue
        out.append(lines[i])
        i += 1
    return '\n'.join(out)


def nav_items(nav):
    for entry in nav:
        for title, value in entry.items():
            if isinstance(value, list):
                yield title, None
                yield from nav_items(value)
            else:
                yield title, value


def main():
    wiki = sys.argv[1]
    with open(os.path.join(ROOT, 'mkdocs.yml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    docs = os.path.join(ROOT, 'docs')
    for name in sorted(os.listdir(docs)):
        if not name.endswith('.md'):
            continue
        with open(os.path.join(docs, name), encoding='utf-8') as f:
            text = convert(f.read())
        text += ('\n\n---\n*Mirrored from the [documentation site](%s); edit `docs/%s` in the repository.*\n'
                 % (cfg.get('site_url', ''), name))
        with open(os.path.join(wiki, page_name(name) + '.md'), 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
    side = ['**[%s](Home)**' % cfg['site_name'], '']
    for title, target in nav_items(cfg['nav']):
        if target is None:
            side += ['', '**%s**' % title]
        elif target != 'index.md':
            side.append('- [%s](%s)' % (title, page_name(target)))
    with open(os.path.join(wiki, '_Sidebar.md'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(side) + '\n')
    print('wiki pages written to', wiki)


if __name__ == '__main__':
    main()
