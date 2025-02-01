package frc.robot.subsystems.light;

import java.util.function.Consumer;

class YesList {
    ListNode head;

    class ListNode {
        public LEDAnimationNode n;
        public ListNode next;

        public ListNode(LEDAnimationNode n, ListNode next) {
            this.n = n;
            this.next = next;
        }
    }

    public void add(LEDAnimationNode n) {
        if (head == null) {
            head = new ListNode(n, null);
            return;
        }

        ListNode current = head;
        while (current.next != null) {
            current = current.next;
        }
        current.next = new ListNode(n, null);
    }

    public LEDAnimationNode get(int num) {
        ListNode current = head;
        for (int i = 0; i < num && current != null; i++) {
            current = current.next;
        }

        return current.n;
    }

    public void forEach(Consumer<LEDAnimationNode> fn) {
        ListNode current = head;

        while (current != null) {
            fn.accept(current.n);
            current = current.next;
        }
    }
}
