import json
import os
import uuid
from collections import defaultdict
from datetime import datetime, timezone
from typing import Any

from chatkit.store import NotFoundError, Store
from chatkit.types import Attachment, Page, ThreadItem, ThreadMetadata

STORE_FILE = "chatkit_store.json"

class MyChatKitStore(Store[dict]):
    def __init__(self):
        self.threads: dict[str, ThreadMetadata] = {}
        self.items: dict[str, list[ThreadItem]] = defaultdict(list)
        self._load_from_file()

    def _load_from_file(self):
        if not os.path.exists(STORE_FILE):
            return
        try:
            with open(STORE_FILE, "r") as f:
                data = json.load(f)
                for t_data in data.get("threads", {}).values():
                    meta = ThreadMetadata.model_validate(t_data)
                    # Ensure timezone-aware datetimes
                    if meta.created_at.tzinfo is None:
                        meta.created_at = meta.created_at.replace(tzinfo=timezone.utc)
                    # Handle updated_at if it exists
                    if hasattr(meta, 'updated_at') and meta.updated_at and meta.updated_at.tzinfo is None:
                        meta.updated_at = meta.updated_at.replace(tzinfo=timezone.utc)
                    self.threads[t_data["id"]] = meta
                
                for t_id, items_data in data.get("items", {}).items():
                    items = [ThreadItem.model_validate(i) for i in items_data]
                    for item in items:
                        if hasattr(item, 'created_at') and item.created_at.tzinfo is None:
                            item.created_at = item.created_at.replace(tzinfo=timezone.utc)
                    self.items[t_id] = items
        except Exception as e:
            print(f"Error loading store: {e}")

    def _save_to_file(self):
        data = {
            "threads": {k: v.model_dump(mode="json") for k, v in self.threads.items()},
            "items": {k: [i.model_dump(mode="json") for i in v] for k, v in self.items.items()},
        }
        with open(STORE_FILE, "w") as f:
            json.dump(data, f, indent=2)

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        if thread_id not in self.threads:
            self.threads[thread_id] = ThreadMetadata(
                id=thread_id,
                created_at=datetime.now(timezone.utc),
                updated_at=datetime.now(timezone.utc)
            )
            self._save_to_file()
        return self.threads[thread_id]

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        self.threads[thread.id] = thread
        self._save_to_file()

    async def load_threads(
        self, limit: int, after: str | None, order: str, context: dict
    ) -> Page[ThreadMetadata]:
        threads = list(self.threads.values())
        return self._paginate(
            threads, after, limit, order, sort_key=lambda t: t.created_at, cursor_key=lambda t: t.id
        )

    async def load_thread_items(
        self, thread_id: str, after: str | None, limit: int, order: str, context: dict
    ) -> Page[ThreadItem]:
        items = self.items.get(thread_id, [])
        return self._paginate(
            items, after, limit, order, sort_key=lambda i: i.created_at, cursor_key=lambda i: i.id
        )

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: dict
    ) -> None:
        print(f"\n{'='*60}")
        print(f"📥 add_thread_item called")
        print(f"   Thread: {thread_id}")
        print(f"   Item ID: {item.id}")
        print(f"   Item Type: {item.type}")
        
        # Show first 100 chars of content if it exists
        if hasattr(item, 'content') and item.content:
            content_preview = str(item.content)[:100]
            print(f"   Content: {content_preview}...")
        
        # CRITICAL FIX: Ensure unique ID before adding
        if not item.id:
            item.id = f"{item.type}_{uuid.uuid4().hex[:12]}"
            print(f"⚠️  Generated ID for new item: {item.id}")
        
        # Ensure timezone-aware datetime
        if hasattr(item, 'created_at') and item.created_at and item.created_at.tzinfo is None:
            item.created_at = item.created_at.replace(tzinfo=timezone.utc)
        
        # DEBUG: Check for duplicates
        existing_items = self.items[thread_id]
        print(f"   Current items in thread: {len(existing_items)}")
        for i, ex in enumerate(existing_items):
            print(f"      [{i}] ID={ex.id}, Type={ex.type}")
        
        if item.id in [i.id for i in existing_items]:
            print(f"🚨 WARNING: add_thread_item called with duplicate ID: {item.id}")
            print(f"   Type: {item.type}, Created: {item.created_at}")
            # Generate a new unique ID to prevent overwrite
            old_id = item.id
            item.id = f"{item.type}_{uuid.uuid4().hex[:12]}"
            print(f"   Changed ID: {old_id} → {item.id}")
        
        print(f"✅ Adding item: {item.id} (type={item.type})")
        self.items[thread_id].append(item)
        print(f"   Total items after add: {len(self.items[thread_id])}")
        print(f"{'='*60}\n")
        self._save_to_file()

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: dict
    ) -> None:
        print(f"\n{'='*60}")
        print(f"💾 save_item called")
        print(f"   Thread: {thread_id}")
        print(f"   Item ID: {item.id}")
        print(f"   Item Type: {item.type}")
        
        # Show first 100 chars of content if it exists
        if hasattr(item, 'content') and item.content:
            content_preview = str(item.content)[:100]
            print(f"   Content: {content_preview}...")
        
        # CRITICAL FIX: Ensure unique ID before saving
        if not item.id:
            item.id = f"{item.type}_{uuid.uuid4().hex[:12]}"
            print(f"⚠️  Generated ID for save: {item.id}")
        
        # Ensure timezone-aware datetime
        if hasattr(item, 'created_at') and item.created_at and item.created_at.tzinfo is None:
            item.created_at = item.created_at.replace(tzinfo=timezone.utc)
        
        items = self.items[thread_id]
        print(f"   Current items in thread: {len(items)}")
        
        # Find existing item with same ID
        for idx, existing in enumerate(items):
            if existing.id == item.id:
                print(f"📝 Updating existing item at index {idx}: {item.id} (type={item.type})")
                print(f"   OLD content: {str(getattr(existing, 'content', ''))[:100]}...")
                print(f"   NEW content: {str(getattr(item, 'content', ''))[:100]}...")
                items[idx] = item
                print(f"{'='*60}\n")
                self._save_to_file()
                return
        
        # No existing item found, add as new
        print(f"✅ Adding new item: {item.id} (type={item.type})")
        items.append(item)
        print(f"   Total items after add: {len(items)}")
        print(f"{'='*60}\n")
        self._save_to_file()

    async def load_item(
        self, thread_id: str, item_id: str, context: dict
    ) -> ThreadItem:
        for item in self.items.get(thread_id, []):
            if item.id == item_id:
                return item
        raise NotFoundError(f"Item {item_id} not found in thread {thread_id}")

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        self.threads.pop(thread_id, None)
        self.items.pop(thread_id, None)
        self._save_to_file()

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: dict
    ) -> None:
        self.items[thread_id] = [
            item for item in self.items.get(thread_id, []) if item.id != item_id
        ]
        self._save_to_file()

    def _paginate(self, rows: list, after: str | None, limit: int, order: str, sort_key, cursor_key):
        # Normalize datetimes to be timezone-aware for comparison
        def normalize_sort_key(row):
            dt = sort_key(row)
            # If datetime is naive, make it UTC-aware
            if isinstance(dt, datetime) and dt.tzinfo is None:
                return dt.replace(tzinfo=timezone.utc)
            return dt
        
        sorted_rows = sorted(rows, key=normalize_sort_key, reverse=order == "desc")
        start = 0
        if after:
            for idx, row in enumerate(sorted_rows):
                if cursor_key(row) == after:
                    start = idx + 1
                    break
        data = sorted_rows[start : start + limit]
        has_more = start + limit < len(sorted_rows)
        next_after = cursor_key(data[-1]) if has_more and data else None
        return Page(data=data, has_more=has_more, after=next_after)

    # Attachments are intentionally not implemented for the quickstart

    async def save_attachment(
        self, attachment: Attachment, context: dict
    ) -> None:
        raise NotImplementedError()

    async def load_attachment(
        self, attachment_id: str, context: dict
    ) -> Attachment:
        raise NotImplementedError()

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        raise NotImplementedError()