import os
from dotenv import load_dotenv
from agents import OpenAIChatCompletionsModel, RunConfig
from openai import AsyncOpenAI

load_dotenv()

# Base URL + Key
base_url = os.getenv('GEMINI_BASE_URL')

api_key = os.getenv('GEMINI_API_KEY')

# Main model
model_name = os.getenv('GEMINI_MODEL_NAME')


if not base_url or not api_key:
    raise ValueError("GEMINI_BASE_URL or GEMINI_API_KEY environment variable is not set.")

external_client = AsyncOpenAI(
    api_key=api_key,
    base_url=base_url,
)

"""
Exports gemini model:
- gemini_model (main)
"""

# Main Gemini model
gemini_model = OpenAIChatCompletionsModel(
    model=model_name,
    openai_client=external_client,
)


# Default run config uses main model
config = RunConfig(
    model=gemini_model,
    model_provider=external_client,
    tracing_disabled=True,
)
