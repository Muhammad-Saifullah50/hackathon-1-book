from typing import AsyncIterator
from chatkit.server import ChatKitServer
from chatkit.types import UserMessageItem, ThreadMetadata
from chatkit.agents import AgentContext, stream_agent_response, simple_to_agent_input
from agents import Runner
from agent import rag_tutor_agent
from chatkit_store import MyChatKitStore


class RagTutorChatKitServer(ChatKitServer):
    def __init__(self):
        # Pass MyChatKitStore to the parent constructor using keyword argument
        super().__init__(store=MyChatKitStore())
        self.agent = rag_tutor_agent

    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: dict,
    ) -> AsyncIterator:
        # Convert recent thread items (which includes the user message) to model input
        items_page = await self.store.load_thread_items(
            thread.id,
            after=None,
            limit=20,
            order="asc",
            context=context,
        )
        input_items = await simple_to_agent_input(items_page.data)

        # Stream the run through ChatKit events
        agent_context = AgentContext(
            thread=thread, store=self.store, request_context=context
        )
        result = Runner.run_streamed(
            rag_tutor_agent, input_items, context=agent_context
        )
        async for event in stream_agent_response(agent_context, result):
            yield event
