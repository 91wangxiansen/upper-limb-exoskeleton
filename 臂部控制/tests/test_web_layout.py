"""Markup regressions for the compact, display-only twin workspace."""
from html.parser import HTMLParser
from pathlib import Path
import re
import unittest

WEB = Path(__file__).resolve().parents[1] / 'web'


class LayoutParser(HTMLParser):
    VOID = {'meta', 'link', 'input', 'br', 'hr', 'img', 'source', 'area', 'base', 'embed', 'param', 'wbr'}

    def __init__(self, source):
        super().__init__()
        self.stack = []
        self.nodes = []
        self.errors = []
        self.feed(source)

    def handle_starttag(self, tag, attrs):
        node = {'tag':tag, 'attrs':dict(attrs), 'parents':list(self.stack)}
        self.nodes.append(node)
        if tag not in self.VOID:
            self.stack.append(node)

    def handle_endtag(self, tag):
        if not self.stack or self.stack[-1]['tag'] != tag:
            self.errors.append(tag)
        else:
            self.stack.pop()


class TwinWorkspaceLayoutTests(unittest.TestCase):
    def setUp(self):
        self.layout = LayoutParser((WEB / 'index.html').read_text(encoding='utf-8'))

    def by_id(self, key):
        return next(node for node in self.layout.nodes if node['attrs'].get('id') == key)

    @staticmethod
    def has_parent(node, name):
        return any(name in parent['attrs'].get('class','').split() for parent in node['parents'])

    def test_balanced_markup_and_unique_ids(self):
        self.assertEqual(self.layout.errors, [])
        self.assertEqual(self.layout.stack, [])
        ids = [node['attrs']['id'] for node in self.layout.nodes if 'id' in node['attrs']]
        self.assertEqual(len(ids), len(set(ids)))

    def test_all_charts_are_in_separate_left_sidebar_not_model_column(self):
        for key in ('positionChart','velocityChart','torqueChart','temperatureChart'):
            self.assertTrue(self.has_parent(self.by_id(key),'telemetry-column'))
            self.assertFalse(self.has_parent(self.by_id(key),'visual-column'))
        self.assertTrue(self.has_parent(self.by_id('viewport'),'visual-column'))

    def test_primary_action_is_outside_scrolling_settings_and_details(self):
        for key in ('syncMoveButton','stopButton'):
            self.assertFalse(self.has_parent(self.by_id(key),'control-scroll'))
            self.assertFalse(any(parent['tag']=='details' for parent in self.by_id(key)['parents']))
        folds = [node for node in self.layout.nodes if node['tag']=='details']
        self.assertGreaterEqual(len(folds),7)
        self.assertTrue(all('open' not in node['attrs'] for node in folds))

    def test_app_registry_targets_exist_and_command_inputs_preserved(self):
        app = (WEB/'app.js').read_text(encoding='utf-8')
        registry = re.search(r'const ui = Object.fromEntries\(\[(.*?)\]\.map',app,re.S).group(1)
        for key in re.findall(r"'([^']+)'",registry):
            self.by_id(key)
        for joint in ('shoulder','elbow'):
            inputs = [node['attrs']['data-input'] for node in self.layout.nodes
                      if 'data-input' in node['attrs'] and any(parent['attrs'].get('data-joint')==joint for parent in node['parents'])]
            self.assertEqual(set(inputs), {'direction','minimum','maximum','maxSpeed','maxTorque','kp','kd','target','targetNumber','speed','torque'})
            self.assertEqual(len(inputs),len(set(inputs)))


if __name__ == '__main__':
    unittest.main()
