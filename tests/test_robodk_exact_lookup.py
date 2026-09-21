import unittest
from unittest.mock import Mock
from robodk_bridge import _find_exact_items


class ExactLookupTests(unittest.TestCase):
    def test_bulk_name_lookup_keeps_exact_match(self):
        rdk=Mock(); rdk.ItemList.return_value=['Target_'+str(i) for i in range(5000)]
        item=Mock(); item.Valid.return_value=True; item.Name.return_value='Target_20'
        rdk.Item.return_value=item
        self.assertEqual(_find_exact_items(rdk,'Target_20',6,'target'),[item])
        rdk.ItemList.assert_called_once_with(6,True)
        item.Name.assert_called_once()

    def test_duplicates_are_still_rejected(self):
        rdk=Mock(); rdk.ItemList.return_value=['A','A']
        with self.assertRaisesRegex(RuntimeError,'Duplicate'):
            _find_exact_items(rdk,'A',6,'target')
        rdk.Item.assert_not_called()

    def test_no_fuzzy_fallback_after_concurrent_change(self):
        rdk=Mock(); rdk.ItemList.return_value=['A']
        rdk.Item.return_value.Valid.return_value=True
        rdk.Item.return_value.Name.return_value='A_old'
        self.assertEqual(_find_exact_items(rdk,'A',6,'target'),[])


if __name__=='__main__': unittest.main()
