import { connect } from '../../../web/index.js';

const endpoint = process.argv[2] || 'http://127.0.0.1:9001';
let ros;

try {
  ros = await connect(endpoint);
  const goal = await ros.action(
    '/fibonacci',
    { order: 5 },
    {
      onFeedback(feedback) {
        console.log('Feedback:', feedback.sequence);
      },
    }
  );
  const result = await goal.result;
  console.log('Status:', goal.status);
  console.log('Result:', result.sequence);
  if (goal.status !== 'succeeded') process.exitCode = 1;
} catch (error) {
  console.error(`${error.code || 'action_failed'}: ${error.message}`);
  process.exitCode = 1;
} finally {
  await ros?.close();
}
