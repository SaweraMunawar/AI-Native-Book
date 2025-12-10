/**
 * API client for chat endpoints
 */

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export interface Source {
  chapter_slug: string;
  chapter_title: string;
  section_id?: string;
  section_title?: string;
  excerpt: string;
  score: number;
}

export interface ChatResponse {
  message_id: string;
  session_id: string;
  answer: string;
  confidence: 'high' | 'medium' | 'low';
  sources: Source[];
  disclaimer?: string;
}

export interface ChatRequest {
  message: string;
  session_id?: string;
}

export interface ContextualChatRequest extends ChatRequest {
  selected_text: string;
  chapter_slug?: string;
}

export interface ApiError {
  error: string;
  code: string;
  details?: Record<string, unknown>;
}

/**
 * Send a chat message to the API
 */
export async function sendMessage(request: ChatRequest): Promise<ChatResponse> {
  const response = await fetch(`${API_URL}/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.error || 'Failed to send message');
  }

  return response.json();
}

/**
 * Send a contextual chat message with selected text
 */
export async function sendContextualMessage(
  request: ContextualChatRequest
): Promise<ChatResponse> {
  const response = await fetch(`${API_URL}/chat/context`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.error || 'Failed to send message');
  }

  return response.json();
}

/**
 * Check if the API is available
 */
export async function checkHealth(): Promise<boolean> {
  try {
    const response = await fetch(`${API_URL}/health`, {
      method: 'GET',
    });
    return response.ok;
  } catch {
    return false;
  }
}
