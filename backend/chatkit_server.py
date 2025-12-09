from typing import AsyncIterator, Any
from chatkit.server import ChatKitServer
from chatkit.types import UserMessageItem, ThreadMetadata, ClientToolCallItem
from chatkit.agents import AgentContext, stream_agent_response, simple_to_agent_input
from agents import Runner
from agent import rag_tutor_agent
from chatkit_store import MyChatKitStore
import uuid


class RagTutorChatKitServer(ChatKitServer):
    def __init__(self):
        super().__init__(store=MyChatKitStore())
        self.agent = rag_tutor_agent

    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | ClientToolCallItem | None,
        context: Any,
    ) -> AsyncIterator:
        # Create agent context - this gives the agent access to the store
        # The agent/store will handle history loading internally
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Convert ONLY the new input item to agent format
        # Don't manually load or pass history - AgentContext handles that
        new_items = await simple_to_agent_input(input) if input else []

        # Run the agent with only the new input
        # The agent will fetch history through AgentContext as needed
        result = Runner.run_streamed(
            self.agent,
            new_items,
            context=agent_context,
        )

        # CRITICAL FIX: Intercept events and fix IDs before streaming
        async for event in stream_agent_response(agent_context, result):
            # If this is a message event with a fake ID, replace it
            if hasattr(event, 'item') and hasattr(event.item, 'id'):
                if event.item.id == '__fake_id__' or not event.item.id:
                    event.item.id = f"msg_{uuid.uuid4().hex[:12]}"
                    print(f"🔧 Fixed fake ID → {event.item.id}")
            
            yield event