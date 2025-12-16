from typing import AsyncIterator, Any
import asyncio
from chatkit.server import ChatKitServer
from chatkit.types import UserMessageItem, ThreadMetadata, ClientToolCallItem
from chatkit.agents import AgentContext, stream_agent_response, simple_to_agent_input
from agents import Runner, Agent
from agent import rag_tutor_agent
from models.gemini import gemini_model
from src.services.chat_store import NeonChatKitStore
import uuid
import os
from dotenv import load_dotenv

load_dotenv()

# Title generation agent - creates concise thread titles from user's first message
title_agent = Agent(
    model=gemini_model,
    name="Thread Title Generator",
    instructions="""You generate concise, descriptive titles for chat conversations.
    Based on the user's first message, create a short title (3-6 words) that captures the main topic.

    Examples:
    - User: "Can you explain ROS 2?" → Title: "ROS 2 Explanation"
    - User: "How do I set up Isaac Sim?" → Title: "Isaac Sim Setup"
    - User: "What is VLA architecture?" → Title: "VLA Architecture Overview"

    Return ONLY the title text, nothing else."""
)


class RagTutorChatKitServer(ChatKitServer):
    def __init__(self):
        # Use Neon database store for persistent chat history
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            raise ValueError("DATABASE_URL environment variable not set")

        super().__init__(store=NeonChatKitStore(database_url))
        self.agent = rag_tutor_agent

    async def maybe_update_thread_title(
        self,
        thread: ThreadMetadata,
        input_item: UserMessageItem,
        context: Any,
    ) -> None:
        """
        Generate a title for new threads based on the first user message.

        Args:
            thread: Thread metadata
            input_item: User's message item
            context: Request context
        """
        # Only generate title if thread doesn't have one
        if thread.title is not None:
            return

        try:
            # Convert user message to agent input
            agent_input = await simple_to_agent_input(input_item)

            # Run title generation agent
            run = await Runner.run(title_agent, input=agent_input)

            # Update thread with generated title
            thread.title = run.final_output.strip()

            # Save the updated thread
            await self.store.save_thread(thread, context)

            print(f"✨ Generated thread title: '{thread.title}' for thread {thread.id}")
        except Exception as e:
            print(f"⚠️  Failed to generate thread title: {e}")
            # Don't fail the request if title generation fails

    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | ClientToolCallItem | None,
        context: Any,
    ) -> AsyncIterator:
        # Generate thread title asynchronously (non-blocking)
        # This runs in the background and doesn't delay the response
        if input and isinstance(input, UserMessageItem):
            asyncio.create_task(self.maybe_update_thread_title(thread, input, context))

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

        # Stream agent response - ChatKit handles saving to store automatically
        async for event in stream_agent_response(agent_context, result):
            yield event